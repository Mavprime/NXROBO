#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spark Autonomous Robot - Comprehensive Hardware & Subsystem Diagnostic Tool
Executes non-destructive health checks across all hardware sensors, ROS topics,
serial ports, and running processes.
"""

import sys
import os
import subprocess
import time
import socket


class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def print_header(title):
    print("\n" + Colors.CYAN + "=" * 65 + Colors.RESET)
    print(f"{Colors.BOLD}{Colors.CYAN} 🔍 {title}{Colors.RESET}")
    print(Colors.CYAN + "=" * 65 + Colors.RESET)


def report(test_name, status, details=""):
    if status is True:
        tag = f"{Colors.GREEN}[ PASS ]{Colors.RESET}"
    elif status is False:
        tag = f"{Colors.RED}[ FAIL ]{Colors.RESET}"
    else:
        tag = f"{Colors.YELLOW}[ WARN ]{Colors.RESET}"
    print(f" {tag} {test_name:<38} {details}")


def run_cmd(cmd, timeout=3):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return res.returncode == 0, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Timeout expired"
    except Exception as e:
        return False, "", str(e)


def check_calibration_files():
    print_header("1. ПРОВЕРКА ФАЙЛОВ ЗАВОДСКОЙ КАЛИБРОВКИ")
    home = os.environ.get("HOME", "/home/spark")

    for fpath, name in [
        (os.path.join(home, "thefile.txt"), "Калибровка 1-го яруса (thefile.txt)"),
        (os.path.join(home, "thefile2.txt"), "Калибровка 2-го яруса (thefile2.txt)")
    ]:
        if os.path.exists(fpath):
            with open(fpath, "r") as f:
                content = f.read().split()
            if len(content) >= 4:
                report(name, True, f"Коэффициенты: k1={content[0]}, b1={content[1]}, k2={content[2]}, b2={content[3]}")
            else:
                report(name, False, "Файл поврежден (меньше 4 чисел)")
        else:
            report(name, False, f"Файл не найден по пути: {fpath}")


def check_processes():
    print_header("2. СТАТУС СИСТЕМНЫХ ПРОЦЕССОВ ROS")
    nodes = [
        ("roscore / master", "roscore|rosmaster"),
        ("Шасси и камера (spark_base)", "spark_base_server"),
        ("Манипулятор uArm (swiftpro)", "pro_control_nomoveit"),
        ("Веб-видеострим (8080)", "camera_stream.py"),
        ("Детектор объектов (detect_ros)", "detect_ros.py"),
        ("Конечный автомат (runtask)", "runtask.py"),
    ]

    for label, proc_pattern in nodes:
        ok, out, _ = run_cmd(f"pgrep -f '{proc_pattern}'")
        if ok and out:
            pids = out.replace('\n', ', ')
            report(label, True, f"PID: {pids}")
        else:
            report(label, False, "Процесс не запущен")


def check_serial_ports():
    print_header("3. СЕРИЙНЫЕ ПОРТЫ И УСТРОЙСТВА")
    # uArm serial
    ok, out, _ = run_cmd("ls -l /dev/ttyACM* 2>/dev/null")
    if ok and out:
        report("Порт манипулятора (/dev/ttyACM*)", True, out.split()[-1])
    else:
        report("Порт манипулятора (/dev/ttyACM*)", False, "Устройство не подключено или нет прав")

    # Lidar serial
    ok, out, _ = run_cmd("ls -l /dev/ydlidar* /dev/ttyUSB* 2>/dev/null")
    if ok and out:
        first_line = out.split('\n')[0]
        report("Порт лидара (/dev/ttyUSB* /ydlidar)", True, first_line.split()[-1])
    else:
        report("Порт лидара (/dev/ydlidar)", None, "Специфический симлинк не найден (проверьте dmesg)")


def check_web_stream():
    print_header("4. СЕТЕВОЙ ВИДЕОСТРИМ MJPEG")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.5)
    result = sock.connect_ex(('127.0.0.1', 8080))
    sock.close()

    if result == 0:
        report("Веб-сервер камеры (порт 8080)", True, "Слушает входящие соединения на http://<IP>:8080/")
    else:
        report("Веб-сервер камеры (порт 8080)", False, "Порт 8080 закрыт (camera_stream.py не активен)")


def check_ros_topics():
    print_header("5. ЧАСТОТА ТОПИКОВ ROS И СЕНСОРОВ")
    # Quick probe if roscore is responsive
    ok, _, _ = run_cmd("source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; rostopic list", timeout=3)
    if not ok:
        report("Связь с ROS Master", False, "roscore недоступен")
        return

    topics_to_check = [
        ("/camera/color/image_raw", "Камера RealSense D435", 5),
        ("/scan", "Лазерный дальномер (Лидар)", 5),
        ("/spark_base/sensor", "Датчики шасси и бамперы", 1),
    ]

    for topic, label, expected_hz in topics_to_check:
        cmd = f"source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; rostopic hz -w 2.5 {topic} 2>&1"
        ok, out, _ = run_cmd(cmd, timeout=3.5)
        if "average rate:" in out:
            rate_line = [l for l in out.split('\n') if "average rate:" in l][0]
            rate_val = rate_line.split("average rate:")[-1].strip().split()[0]
            report(f"Топик {topic}", True, f"{rate_val} Hz ({label})")
        else:
            # Fallback probe: check if single message can be echoed
            cmd_echo = f"source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; rostopic echo -n 1 --noarr {topic} 2>&1"
            ok_echo, out_echo, _ = run_cmd(cmd_echo, timeout=3.0)
            if ok_echo and len(out_echo) > 5 and "ERROR" not in out_echo:
                report(f"Топик {topic}", True, f"Активен (данные поступают) ({label})")
            else:
                report(f"Топик {topic}", False, f"Нет данных с сенсора ({label})")

    # Check IR Bumper specifically
    cmd_bumper = "source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; rostopic echo -n 1 /spark_base/sensor 2>/dev/null"
    ok, out, _ = run_cmd(cmd_bumper, timeout=2.0)
    if ok and out:
        is_bumper_stuck = "ir_bumper_left: True" in out or "ir_bumper_right: True" in out
        if is_bumper_stuck:
            report("Статус ИК-бамперов базы", False, "ВНИМАНИЕ: Бампер зажат! Шасси блокирует движение.")
        else:
            report("Статус ИК-бамперов базы", True, "Бамперы свободны (движение разрешено)")
    else:
        report("Статус ИК-бамперов базы", None, "Не удалось прочитать /spark_base/sensor")


def main():
    print(f"{Colors.BOLD}{Colors.YELLOW}")
    print("=================================================================")
    print("   NXROBO SPARK ROBOT: КОМПЛЕКСНАЯ ДИАГНОСТИКА ПОДСИСТЕМ        ")
    print("=================================================================")
    print(f"{Colors.RESET}")

    check_calibration_files()
    check_processes()
    check_serial_ports()
    check_web_stream()
    check_ros_topics()

    print("\n" + Colors.CYAN + "=" * 65 + Colors.RESET)
    print(f"{Colors.BOLD}Диагностический прогон завершен.{Colors.RESET}")
    print(Colors.CYAN + "=" * 65 + Colors.RESET + "\n")


if __name__ == "__main__":
    main()

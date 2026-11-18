#!/usr/bin/env bash
# Единый интерактивный пульт управления Spark
# С автопоиском скрипта телеуправления и защитой от смены имен файлов

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null
mkdir -p ~/logs
touch ~/logs/runtask.log ~/logs/base.log ~/logs/arm.log ~/logs/stream.log ~/logs/detect.log

while true; do
    trap '' INT  # Блокируем Ctrl+C в самом меню, чтобы случайно не вылететь
    clear
    ROBOT_IP=$(hostname -I | awk '{print $1}')
    echo "======================================================"
    echo "       SPARK ROBOT: ЕДИНОЕ МЕНЮ УПРАВЛЕНИЯ            "
    echo "       IP: ${ROBOT_IP} | Веб-стрим: :8080             "
    echo "======================================================"
    echo "  Статус фоновых служб:"
    pgrep -f "spark_base_server" >/dev/null && echo "    [✓] База и RealSense D435:  РАБОТАЕТ" || echo "    [✗] База: ОСТАНОВЛЕНА"
    pgrep -f "pro_control_nomoveit" >/dev/null && echo "    [✓] Манипулятор uArm:      РАБОТАЕТ" || echo "    [✗] Рука: ОСТАНОВЛЕНА"
    pgrep -f "camera_stream.py" >/dev/null && echo "    [✓] Стрим камеры :8080:     РАБОТАЕТ" || echo "    [✗] Стрим: ОСТАНОВЛЕН"
    pgrep -f "detect_ros.py" >/dev/null && echo "    [✓] Нейросеть-детектор:    РАБОТАЕТ" || echo "    [✗] Детектор: ОСТАНОВЛЕН"
    pgrep -f "runtask.py" >/dev/null && echo "    [✓] Мозг runtask:          РАБОТАЕТ" || echo "    [✗] runtask: ОСТАНОВЛЕН"
    echo "------------------------------------------------------"
    echo "  1) ЗАПУСТИТЬ ВСЕ СИСТЕМЫ (start_system.sh)"
    echo "  2) ТЕСТ ЗАХВАТА КУБИКА (рука хватает кубик и поднимает)"
    echo "  3) ТЕСТ ОТПУСКАНИЯ КУБИКА (помпа отпускает)"
    echo "  4) ПЛАВНОЕ УПРАВЛЕНИЕ С КЛАВИАТУРЫ (автопоиск key*.py/teleop*.py)"
    echo "  5) СМОТРЕТЬ ЖИВОЙ ЛОГ RUNTASK (нажмите 'q' для выхода)"
    echo "  6) ОСТАНОВИТЬ ВСЕ СИСТЕМЫ (stop_system.sh)"
    echo "  0) Выход в bash"
    echo "======================================================"
    read -p "Выберите действие [0-6]: " choice

    case $choice in
        1)
            bash ~/codes/start_system.sh
            echo ""
            read -p "Нажмите Enter для возврата в меню..."
            ;;
        2)
            echo ">> Отправка команды на захват кубика..."
            rostopic pub -1 /grasp std_msgs/String "data: '1'" >/dev/null 2>&1
            echo ">> Команда отправлена. Последние записи runtask:"
            sleep 1
            tail -n 8 ~/logs/runtask.log
            echo ""
            read -p "Нажмите Enter для возврата в меню..."
            ;;
        3)
            echo ">> Отправка команды на отпускание..."
            rostopic pub -1 /grasp std_msgs/String "data: '0'" >/dev/null 2>&1
            echo ">> Команда отправлена. Последние записи runtask:"
            sleep 1
            tail -n 6 ~/logs/runtask.log
            echo ""
            read -p "Нажмите Enter для возврата в меню..."
            ;;
        4)
            trap - INT  # Возвращаем Ctrl+C для управления клавиатурой
            # Автопоиск самого свежего скрипта телеуправления:
            TELEOP_SCRIPT=$(ls -t ~/codes/key*.py ~/codes/*teleop*.py 2>/dev/null | head -n 1)
            if [ -n "$TELEOP_SCRIPT" ] && [ -f "$TELEOP_SCRIPT" ]; then
                echo "Запуск скрипта телеуправления: $TELEOP_SCRIPT"
                python3 "$TELEOP_SCRIPT"
            else
                echo "Скрипт управления не найден в ~/codes/. Положите файл вида key*.py или *teleop*.py"
                sleep 2
            fi
            ;;
        5)
            echo ">> Просмотр лога. Чтобы вернуться в меню, нажмите клавишу 'q':"
            sleep 1
            less +F ~/logs/runtask.log
            ;;
        6)
            bash ~/codes/stop_system.sh
            echo ""
            read -p "Нажмите Enter для возврата в меню..."
            ;;
        0)
            trap - INT
            break
            ;;
        *)
            echo "Неверный выбор."
            sleep 1
            ;;
    esac
done

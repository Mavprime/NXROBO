import os
import sys
import time
import paramiko

ROBOT_IP = "10.24.170.0"
ROBOT_USER = "spark"
ROBOT_PASS = "simple2026"

def run_remote(client, cmd, desc=""):
    print(f"\n>> [CMD] {desc or cmd}")
    stdin, stdout, stderr = client.exec_command(f"bash -c \"source /opt/ros/noetic/setup.bash; source ~/spark_noetic/devel/setup.bash; {cmd}\"")
    exit_code = stdout.channel.recv_exit_status()
    out = stdout.read().decode('utf-8', errors='replace').strip()
    err = stderr.read().decode('utf-8', errors='replace').strip()
    if out:
        print(out)
    if err and exit_code != 0:
        print(f"[STDERR] {err}")
    return exit_code, out, err

def main():
    print("=" * 60)
    print("  SPARK ROBOT: DEEP HARDWARE & ROS DIAGNOSTICS  ")
    print("=" * 60)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    priv_key_path = os.path.expanduser(r"~/.ssh/id_ed25519")
    try:
        if os.path.exists(priv_key_path):
            k = paramiko.Ed25519Key.from_private_key_file(priv_key_path)
            client.connect(hostname=ROBOT_IP, port=22, username=ROBOT_USER, pkey=k, timeout=5.0)
        else:
            client.connect(hostname=ROBOT_IP, port=22, username=ROBOT_USER, password=ROBOT_PASS, timeout=5.0)
        print(">> [OK] Connected via SSH.")
    except Exception as e:
        print(f">> [FAIL] SSH connection failed: {e}")
        return 1

    # 1. Check USB devices
    run_remote(client, "lsusb", "List all connected USB devices")

    # 2. Check running processes
    run_remote(client, "ps aux | grep -E 'python|spark|realsense|ydlidar|swiftpro' | grep -v grep", "Process listing")

    # 3. Check ROS nodes
    run_remote(client, "rosnode list", "List active ROS nodes")

    # 4. Check ROS topics for camera and detector
    run_remote(client, "rostopic list | grep -E 'camera|image|objects|scan|grasp'", "Active camera/sensor topics")

    # 5. Check if /camera/color/image_raw has frames
    run_remote(client, "rostopic hz -w 2 /camera/color/image_raw", "Sample /camera/color/image_raw frequency (2s window)")

    # 6. Check logs: stream.log and base.log
    run_remote(client, "tail -n 25 ~/logs/stream.log", "Tail of ~/logs/stream.log")
    run_remote(client, "grep -iE 'error|warn|fail|fatal|camera|d435' ~/logs/base.log | tail -n 25", "Errors/warnings in base.log")

    # 7. Check if detect_ros.py is actually publishing
    run_remote(client, "rostopic hz -w 2 /objects", "Sample /objects topic frequency (2s window)")

    client.close()
    return 0

if __name__ == '__main__':
    sys.exit(main())

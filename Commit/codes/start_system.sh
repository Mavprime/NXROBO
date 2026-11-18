#!/usr/bin/env bash
# Единый фоновый запуск всех подсистем робота Spark на ROS Noetic

mkdir -p ~/logs
touch ~/logs/runtask.log ~/logs/base.log ~/logs/arm.log ~/logs/stream.log ~/logs/detect.log

echo "=========================================="
echo "    Запуск подсистем робота Spark         "
echo "=========================================="

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null

# 1. Запуск драйвера базы, камеры RealSense и лидара
if ! pgrep -f "spark_base_server" > /dev/null; then
    echo "[1/5] Запуск базы, RealSense D435 и YDLidar..."
    nohup roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6 > ~/logs/base.log 2>&1 &
    sleep 4
else
    echo "[1/5] База и камера уже запущены."
fi

# 2. Запуск манипулятора uArm
if ! pgrep -f "pro_control_nomoveit" > /dev/null; then
    echo "[2/5] Запуск драйвера манипулятора uArm..."
    nohup roslaunch swiftpro pro_control_nomoveit.launch > ~/logs/arm.log 2>&1 &
    sleep 3
else
    echo "[2/5] Манипулятор uArm уже запущен."
fi

# 3. Запуск веб-стрима камеры (-u для мгновенного вывода в лог)
fuser -k 8080/tcp 2>/dev/null
echo "[3/5] Запуск веб-стрима камеры на порту 8080..."
nohup python3 -u /home/spark/codes/camera_stream.py > ~/logs/stream.log 2>&1 &
sleep 1

# 4. Запуск детектора нейросетью (-u для мгновенного вывода в лог)
if ! pgrep -f "detect_ros.py" > /dev/null; then
    echo "[4/5] Запуск детектора YOLO/SSD (detect_ros.py)..."
    nohup python3 -u ~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py > ~/logs/detect.log 2>&1 &
    sleep 2
else
    echo "[4/5] Детектор нейросетью уже запущен."
fi

# 5. Запуск runtask (-u для мгновенного вывода в лог)
if ! pgrep -f "runtask.py" > /dev/null; then
    echo "[5/5] Запуск управляющего узла runtask.py..."
    nohup python3 -u ~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py > ~/logs/runtask.log 2>&1 &
    sleep 2
else
    echo "[5/5] runtask.py уже запущен."
fi

ROBOT_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "=========================================="
echo "  Все системы успешно запущены в фоне!    "
echo "  Стрим камеры: http://${ROBOT_IP}:8080/ "
echo "  Логи лежат в папке ~/logs/              "
echo "=========================================="

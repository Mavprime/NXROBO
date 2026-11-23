source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
node_up() { rosnode ping -c 1 "$1" 2>/dev/null | grep -q "reply from"; }
topic_has_msg() { timeout "$1" rostopic echo -n 1 "$2" >/dev/null 2>&1; }
echo "=== START DETECTOR WITH LOW CAKE THRESHOLD (measurement only) $(date +%H:%M:%S) ==="
pkill -f detect_ros.py 2>/dev/null
sleep 1
setsid nohup python3 -u /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py _augment:=true _conf:=0.25 _cake_conf:=0.03 > ~/logs/detect_lowcake.log 2>&1 < /dev/null &
for i in $(seq 1 90); do
    if node_up /detector_node && topic_has_msg 2 /objects; then break; fi
    sleep 1
done
node_up /detector_node && echo "DETECTOR_UP" || echo "DETECTOR_FAILED"
grep -a "publish_conf" ~/logs/detect_lowcake.log | tail -1
echo ""
echo "=== CAKE PROBE 20 s ==="
python3 -u /tmp/cake_probe.py 20 /home/spark/logs/cake_probe.jsonl
echo ""
echo "=== ANNOTATED VIEW WITH cake_conf=0.03 ==="
python3 -u /home/spark/codes/robot_view.py --no-park --cake-conf 0.03 --conf 0.04
echo "=== DONE $(date +%H:%M:%S) ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
NODES=/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes
node_up() { rosnode ping -c 1 "$1" 2>/dev/null | grep -q "reply from"; }
topic_has_msg() { timeout "$1" rostopic echo -n 1 "$2" >/dev/null 2>&1; }
echo "=== CLEAN STALE PRIVATE PARAMS ==="
rosparam delete /detector_node/cake_conf 2>/dev/null
rosparam delete /detector_node/cake_site_conf 2>/dev/null
rosparam list 2>/dev/null | grep -i "detector_node/cake" || echo "no stale cake params"
echo "=== RESTART DETECTOR (file defaults) $(date +%H:%M:%S) ==="
pkill -f detect_ros.py 2>/dev/null
sleep 1
setsid nohup python3 -u $NODES/detect_ros.py _augment:=true _conf:=0.25 > ~/logs/detect.log 2>&1 < /dev/null &
for i in $(seq 1 90); do
    if node_up /detector_node && topic_has_msg 2 /objects; then break; fi
    sleep 1
done
grep -a "publish_conf" ~/logs/detect.log | tail -1
echo "=== DONE $(date +%H:%M:%S) ==="
NODES=/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes
echo "=== MD5 CODES ==="
md5sum /home/spark/codes/grasp_generic.py /home/spark/codes/match_logic.py /home/spark/codes/detect_ros.py /home/spark/codes/runtask.py
echo "=== MD5 NODES ==="
md5sum $NODES/match_logic.py $NODES/detect_ros.py
echo "=== CAKE GATES IN DEPLOYED FILES ==="
grep -n "CAKE_CONF_THRESHOLD = \|CAKE_PUBLISH_CONF = " /home/spark/codes/match_logic.py /home/spark/codes/detect_ros.py $NODES/match_logic.py $NODES/detect_ros.py
echo "=== STOP OLD DETECTOR (will restart with new gate) ==="
pkill -f detect_ros.py 2>/dev/null
sleep 1
pgrep -af detect_ros.py 2>&1
echo "=== PY_COMPILE ==="
python3 -m py_compile /home/spark/codes/grasp_generic.py /home/spark/codes/match_logic.py /home/spark/codes/detect_ros.py /home/spark/codes/runtask.py && echo PY_COMPILE_OK
echo "=== DONE ==="
echo "=== HOST ==="
hostname; date
echo "=== FILE VERSIONS ==="
ls -l /home/spark/codes/grasp_generic.py /home/spark/codes/run_grasp_test.sh 2>&1
md5sum /home/spark/codes/grasp_generic.py 2>&1
wc -l /home/spark/codes/grasp_generic.py 2>&1
echo "=== GRASP TEST LOG (last 30 runs) ==="
tail -n 30 /home/spark/logs/grasp_generic.jsonl 2>&1
echo "=== RECENT LOG FILES ==="
ls -lt /home/spark/logs 2>&1 | head -25
echo "=== PROCESSES ==="
pgrep -af "grasp_generic|swiftpro|detect_ros|spark_base" 2>&1
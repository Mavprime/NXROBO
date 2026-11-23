timeout 240 bash -l /home/spark/codes/run_grasp_test.sh 2>&1 | tail -n 70
echo "SCRIPT_EXIT: $?"
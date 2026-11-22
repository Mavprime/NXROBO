echo "=== FULL CAKE TEST START $(date +%H:%M:%S) ==="
bash -l /home/spark/codes/run_grasp_test.sh
rc=$?
echo "=== FULL CAKE TEST EXIT: $rc ==="
exit $rc
echo "=== FULL GRASP TEST START $(date +%H:%M:%S) ==="
bash -l /home/spark/codes/run_grasp_test.sh
rc=$?
echo "=== FULL GRASP TEST EXIT CODE: $rc ==="
exit $rc
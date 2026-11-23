echo "=== VIEW MODE START $(date +%H:%M:%S) ==="
bash -l /home/spark/codes/run_grasp_test.sh --view
rc=$?
echo "=== VIEW MODE EXIT: $rc ==="
ls -l /home/spark/logs/latest_view.jpg 2>&1
exit $rc
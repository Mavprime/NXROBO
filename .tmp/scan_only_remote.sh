echo "=== SCAN-ONLY (no motion) START $(date +%H:%M:%S) ==="
bash -l /home/spark/codes/run_grasp_test.sh --scan-only
rc=$?
echo "=== SCAN-ONLY EXIT CODE: $rc ==="
exit $rc
LOG=/tmp/grasp_sigint_run.log
: > "$LOG"
echo "=== SIGINT TEST START $(date +%H:%M:%S) ==="
bash -l /home/spark/codes/run_grasp_test.sh > "$LOG" 2>&1 &
RUNNER=$!
for i in $(seq 1 400); do
    if grep -q "Holding the cube in the raised position" "$LOG"; then break; fi
    if ! kill -0 "$RUNNER" 2>/dev/null; then break; fi
    sleep 0.2
done
if grep -q "Holding the cube in the raised position" "$LOG"; then
    sleep 1.5
    echo ">>> sending SIGINT to grasp_generic.py during the 5 s hold"
    pkill -INT -f "/home/spark/codes/grasp_generic.py"
else
    echo ">>> grasp process ended before the hold; nothing to interrupt"
fi
wait "$RUNNER"
rc=$?
echo "=== RUNNER EXIT CODE: $rc ==="
echo "=== FULL LOG ==="
cat "$LOG"
echo "=== SIGINT TEST END $(date +%H:%M:%S) ==="
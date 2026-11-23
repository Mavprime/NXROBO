echo "=== UPLOADED FILES ==="
ls -l /home/spark/codes/robot_view.py /home/spark/codes/run_grasp_test.sh
md5sum /home/spark/codes/robot_view.py /home/spark/codes/run_grasp_test.sh
echo "=== BASH SYNTAX CHECK ==="
bash -n /home/spark/codes/run_grasp_test.sh && echo BASH_SYNTAX_OK
echo "=== PY COMPILE ==="
python3 -m py_compile /home/spark/codes/robot_view.py && echo ROBOT_VIEW_COMPILE_OK
echo "=== DONE ==="
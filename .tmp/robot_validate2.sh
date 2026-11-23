echo "=== REMOTE FILE ==="
ls -l /home/spark/codes/grasp_generic.py
md5sum /home/spark/codes/grasp_generic.py
wc -l /home/spark/codes/grasp_generic.py
echo "=== PY_COMPILE ==="
python3 -m py_compile /home/spark/codes/grasp_generic.py && echo PY_COMPILE_OK
echo "=== IMPORT + PLAN SMOKE (no motion) ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
cd /home/spark/codes
python3 - <<'PY'
import sys
sys.path.insert(0, '/home/spark/codes')
import grasp_generic as g
floor = g.plan_grasp({'mode': 'floor', 'class': 1, 'xy': [240.0, 0.0],
                      'pixel': [320.0, 270.0], 'z_est': -115.0})
cake = g.plan_grasp({'mode': 'cake', 'class': 2, 'xy': [330.0, 10.0],
                     'pixel': [320.0, 240.0], 'z_est': 20.0}, 40.0)
print('FLOOR_PLAN', floor)
print('CAKE_PLAN', cake)
assert (floor.z_approach, floor.z_touch, floor.z_lift) == (70.0, -58.0, 60.0)
assert (cake.z_approach, cake.z_touch, cake.z_lift) == (75.0, 37.0, 80.0)
assert g.HOLD_SECONDS == 5.0
assert not hasattr(g, 'object_near')
try:
    g.plan_grasp({'mode': 'floor', 'class': 1, 'xy': [140.0, 0.0],
                  'pixel': [320.0, 270.0], 'z_est': -115.0})
    raise AssertionError('unreachable plan was not rejected')
except g.GraspError as exc:
    assert exc.result == 'UNREACHABLE', exc
print('IMPORT_AND_PLAN_SMOKE_OK')
PY
echo "=== DONE ==="
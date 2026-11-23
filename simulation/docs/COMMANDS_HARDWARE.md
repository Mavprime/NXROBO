# Spark hardware command sheet

Tested on the real robot, 2026-11-24. Robot `spark@10.24.170.0` (Spark NUC,
Ubuntu 20.04, ROS Noetic). The hardware stack always runs on **ROS master 11311**;
every simulation in this repo uses 11312 or no ROS. Never point a sim at 11311.

Run this in **every** terminal (or add the last line to `~/.bashrc`):

```bash
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
```

## 1. Bring-up (only from a cold robot)

`driver_bringup` starts the master. It does **not** start the lidar.

```bash
# terminal 1 - base + D435 camera (starts rosmaster)
roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6

# terminal 2 - lidar (separate on purpose)
roslaunch lidar_driver_transfer ydlidar_g6.launch

# terminal 3 - arm
roslaunch swiftpro pro_control_nomoveit.launch
```

## 2. Check the stack

```bash
rosnode list
rostopic hz /scan                              # expect ~10 Hz
rostopic hz /odom                              # expect ~12.5 Hz
rostopic hz /camera/color/image_raw/compressed # expect ~30 Hz
rostopic list | grep -E 'position_write|Swiftpro|pump'
```

## 3. Camera in a browser

```bash
python3 ~/auto/codes/camera_stream.py
```

Open:

- live stream: <http://10.24.170.0:8080/>
- single frame: <http://10.24.170.0:8080/snapshot.jpg>

On the robot's own desktop use `http://127.0.0.1:8080/`.
Ctrl+C stops it. If it reports the address in use: `fuser -k 8080/tcp`
(`pkill` self-matches; `fuser` does not).

## 4. Keyboard teleop

Needs a real TTY, hence `ssh -t`.

```bash
ssh -t spark@10.24.170.0
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 ~/auto/codes/keyboard_teleop.py
```

| keys | action |
|---|---|
| `w` / `s` (or Up/Down) | forward / backward |
| `a` / `d` (or Left/Right) | turn left / right |
| **Space** | **emergency stop** (base + arm) |
| `q` | toggle FAST (arena) / SLOW (fine) |
| `8` / `2` | arm up / down (z) |
| `7` / `9` | arm forward / backward (x) |
| `4` / `6` | arm swing CCW / CW (y) |
| `1` / `3` | wrist CCW / CW |
| `v` / `p` / `c` | hover / pick-touch / carry presets |
| `Enter` | toggle vacuum pump |
| `0` | pump off + arm to home |
| `Ctrl+C` | stop everything and exit |

Base-only vendor teleop: `python3 ~/tools/teleop.py`.

Do **not** run `a2w.py` / `a2cw.py` at the same time as teleop - both publish
`/cmd_vel`.

## 5. Record a session

```bash
mkdir -p ~/logs/bags
rosbag record -O ~/logs/bags/teleop_$(date +%Y%m%d_%H%M%S).bag \
  /cmd_vel /odom /scan /tf /tf_static /camera/color/image_raw/compressed \
  /position_write_topic /SwiftproState_topic /angle4th_topic /pump_topic
```

Stop with **Ctrl+C** (finalises the bag) or from another terminal:

```bash
rosnode kill $(rosnode list | grep record)
```

About 1 MB/s. Check space first with `df -h ~`; if a recording was killed
uncleanly the file keeps a `.bag.active` suffix and is not replayable.

Inspect / replay:

```bash
ls -lt ~/logs/bags | head
rosbag info ~/logs/bags/teleop_YYYYmmdd_HHMMSS.bag
rosbag play ~/logs/bags/teleop_YYYYmmdd_HHMMSS.bag
```

Pull one camera frame out of a bag (run on the robot):

```bash
source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash
python3 - ~/logs/bags/teleop_YYYYmmdd_HHMMSS.bag <<'PY'
import sys, rosbag, cv2
from cv_bridge import CvBridge
b = rosbag.Bag(sys.argv[1]); br = CvBridge()
for _, msg, _ in b.read_messages(topics=["/camera/color/image_raw/compressed"]):
    cv2.imwrite("/tmp/frame.jpg", br.compressed_imgmsg_to_cv2(msg, "bgr8")); break
print("wrote /tmp/frame.jpg")
PY
```

## 6. Recordings so far

| bag | size | duration | notes |
|---|---|---|---|
| `~/logs/bags/teleop_20261124_004929.bag` | 220 MB | ~2 min | first test recording |
| `~/logs/bags/teleop_20261124_005714.bag` | 124.6 MB | 118.6 s | full teleop; 3555 camera frames, 3.05 m path, cake in A at the end |

## 7. Live simulation view of the real run (optional, alongside the bag)

`real_view.py` runs the Level-0B 2D viewer on the robot with the **real** pose
and lidar: `/odom` + `/scan`, map frame resolved from the two wall lines (no
AMCL). It also appends every frame to a playback log, so the session becomes a
simulation recording.

```bash
# start it with the robot at A (wall fix, nearest-A hypothesis)
python3 ~/codes/real_view.py

# or anchor explicitly when starting anywhere else (x y yaw_deg)
python3 ~/codes/real_view.py --anchor 1.064 1.562 0

# different port / scenario
python3 ~/codes/real_view.py --port 8091 --scenario ~/codes/config/arena_map.yaml
```

- browser: <http://10.24.170.0:8091/> (binds 0.0.0.0)
- log: `~/logs/real_view/view_<timestamp>.jsonl` (pose + lidar at 5 Hz)
- it refuses to start if the wall fix is more than 35 cm from A - restart at A
  or pass `--anchor`
- cubes are the scenario's initial layout (the layer does not update on a pick)
- stop with Ctrl+C or the page's STOP button

It does not publish `/cmd_vel`, so it can run during teleop or alongside
`a2cw.py`. The rosbag remains the ground truth for arm state and camera; the
JSONL is the map-frame simulation trace.

## 8. Measure the arena (guided anchors)

Full worksheet and the tape measurements: [`ARENA_MEASURE.md`](ARENA_MEASURE.md)
(also on the robot at `~/codes/robot/ARENA_MEASURE.md`).

Terminal 1 (teleop) and terminal 2 (records the wall-fixed robot pose at each
named spot):

```bash
# T1
ssh -t spark@10.24.170.0
source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 ~/auto/codes/keyboard_teleop.py

# T2
ssh spark@10.24.170.0
source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 -u ~/codes/robot/anchor_log.py          # add --yaw0 -45 if the wall fit is ambiguous
```

At each spot type the name in T2: `class`, `W`, `N`, `S`, `E`, `A_approach`.
Output `~/logs/anchors/anchors_<timestamp>.json`; send it back with the filled
tape table.

## Gotchas

- `ROS_MASTER_URI` is per shell. A shell without it defaults to
  `localhost:11311` and a node can fail with *"Unable to register with master
  node"* even though the master is up.
- `/dev/sparkBase` is a symlink to `ttyUSB0` (set at boot). If `/odom` goes
  silent after a power cycle the base USB re-enumerated:
  `rosnode kill /spark_base_server` then
  `roslaunch spark_base spark_base.launch serialport:=/dev/sparkBase`.
- The SwiftPro driver must be restarted after an arm USB re-enumeration, and
  `/dev/ttyACM0` is hard-coded.
- `~/.ros/log` was over 1 GB on 2026-11-24; `rosclean purge` if disk gets tight.
- Keep the charger out of the drive path - a cable under the wheels corrupts
  odometry.

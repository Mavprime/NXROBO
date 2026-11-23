# Arena measurement — rebuild the map

Goal: make `config/arena_map.yaml` (and therefore the simulation, the mission
configs and the robot's named points) match the real floor. The 2026-11-24
teleop showed the real arena is off: the north wall reads ~+7-17 cm from the
map and the initial wall-fix heading varies by ~20 deg between runs.

Two parts: a **tape measurement** of the static layout and a **guided anchor
run** with the robot for the poses it actually uses.

## Frame

Origin at the **SW inner corner** of the arena (the inner face of the west wall
meets the inner face of the south wall). +x east, +y north. Everything in
metres, to the nearest 5 mm.

The current map draws a 3 x 3 m official layout with the physical wall faces
0.175 m outside it (west/south at -0.175, east/north at 3.175). Those offsets
are what we are checking.

## Part 1 — tape measurements

Fill the "measured" column. Measure to the inner faces of the walls, not the
outer edges. Use the diagonal in both directions to check that the arena is
square (equal diagonals = square).

### Shell

| item | how | current map value | measured |
|---|---|---|---|
| inner width (X) | west inner face to east inner face | 3.350 m | |
| inner depth (Y) | south inner face to north inner face | 3.350 m | |
| diagonal SW→NE | corner to corner | — | |
| diagonal NW→SE | corner to corner | — | |
| wall height | floor to top | 0.40 m | |

### Table (classification table on the west wall)

| item | how | current map value | measured |
|---|---|---|---|
| east face x | from SW corner, along x | 0.33 m | |
| north face y | from SW corner, along y | 1.70 m | |
| south face y | from SW corner, along y | 1.30 m | |
| width (x) | east face - west wall | 0.33 m | |
| depth (y) | north - south | 0.40 m | |
| height | floor to top | 0.20 m | |
| board/cards face | which side the cards face | east | |

### Cube cluster

| item | how | current map value | measured |
|---|---|---|---|
| centre x | middle brick centre | 1.474 m | |
| centre y | middle brick centre | 1.562 m | |
| brick size | side of one cube | 0.10 m | |
| row pitch | centre-to-centre spacing | 0.145 m | |
| rows × cols | | 3 x 3 | |
| cake position | centre of the cluster | (1.500, 1.500) | |
| cake size | | 0.10 m | |

### Collection squares (red mats)

| square | how | current map value | measured centre |
|---|---|---|---|
| A | centre of the red square | (0.225, 2.775) | |
| B | centre | (2.775, 2.775) | |
| C | centre | (2.775, 0.225) | |
| D | centre | (0.225, 0.225) | |
| size | side of the red square | 0.45 m | |

### Other

| item | how | current map value | measured |
|---|---|---|---|
| charger footprint | corners, and whether it is on the floor | not in map | |
| start heading at A | the direction you place the robot, 0 deg = east | -45 deg (toward the centre) | |

## Part 2 — guided anchor run (robot poses)

This records where the robot actually stands, wall-fixed, no AMCL.

**Terminal 1 — teleop**

```bash
ssh -t spark@10.24.170.0
source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 ~/auto/codes/keyboard_teleop.py
```

**Terminal 2 — anchor logger**

```bash
ssh spark@10.24.170.0
source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 -u ~/codes/robot/anchor_log.py
```

It records `A_start` immediately, then waits. At each spot, type a name in
terminal 2 and press Enter:

| drive to | type |
|---|---|
| the classification table, where you stand to read the cards | `class` |
| the pick stand-off in front of the cluster, west side, facing the cluster | `W` |
| same, north side | `N` |
| same, south side | `S` |
| same, east side | `E` |
| where you place the cake in A | `A_approach` |

Ctrl+C finishes. Output: `~/logs/anchors/anchors_<timestamp>.json`.

Notes:
- the position comes from the west/east and north/south wall ranges; the
  `span` fields in each record show the disagreement (should be < 0.10 m)
- keep a clear line to the walls on the two axes you need; the lidar is
  mounted high and does not see the table top or the short floor cubes
- if the initial wall fit is ambiguous (it has been every run so far), start it
  with the intended heading instead:
  `python3 -u ~/codes/robot/anchor_log.py --yaw0 -45`

## Results — 2026-11-24 anchor run

Raw file: [`../robot/anchors_20261124.json`](../robot/anchors_20261124.json).
Positions recomputed from the longest wall lines (the `x`/`y` fields written by
`anchor_log.py` use `dist()`, whose north/south ranges are unreliable near the
cluster; the line fits agree with it in `x` and are ~5-10 cm different in `y`).

| anchor | x | y | note |
|---|---|---|---|
| A_start | 0.254 | 2.764 | 3 cm from the map A |
| W | 1.050 | 1.481 | own y is 8 cm low vs the other three |
| N | 1.480 | 1.985 | |
| E | 1.915 | 1.575 | |
| S | 1.450 | 1.206 | |

Fitted cluster centre **1.474, 1.562** (±0.05 m), stand-off 0.41 m. Map updated:

- `Sorting_N (1.474, 1.972)`, `S (1.474, 1.152)`, `E (1.884, 1.562)`,
  `W (1.064, 1.562)`
- bricks and cake re-centred on the new cluster centre
- `cluster_center` added to `arena_map.yaml`; `resolve_goal` and the spatial
  sim now take the Sorting heading from it, not the arena centre
- `scenario.json` / `arena.world` regenerated; 80 tests and both missions pass
  (A-class-W-cake-A score 3, full competition score 21, 0 collisions)

Still open: the wall model and the table were not tape-measured — the north wall
reads up to ~10 cm beyond the map's 3.175, so absolute y may shift once the tape
numbers are in. `class` and `A_approach` anchors were not recorded.

## Then

Send back `~/logs/anchors/anchors_<timestamp>.json` plus the filled tables.
I will update `config/arena_map.yaml` (walls, table, bricks, points), run
`codes/make_arena.py` to regenerate `scenario.json` / `arena.world`, and update
the anchors (`robot/anchor_*.json`) and any tests that hard-code the old
coordinates.

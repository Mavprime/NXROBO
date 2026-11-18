# Spark-H Dual-Track Competition Plan

**Team:** 6 people — Squad A (semi-auto, 3) + Squad B (full-auto, 3)
**Hardware:** 1 × Spark-H (full set: chassis, uArm Swift Pro arm + suction, RealSense D435, YDLidar G6, spark-HM-PI board, HarmonyOS phone)
**Tracks:** M-Robots OS 竞技组 (semi-auto) → then 开源鸿蒙应用赛 (full-auto)

---

## 1. Goal

Run **both** competition tasks on the **same physical Spark-H**, without buying a second robot, without either squad blocking the other, and without a last-minute environment rebuild on match day.

Success = on match day, one documented procedure flips the robot between the two software stacks in under 10 minutes, and each squad can execute its full run from a written runbook.

---

## 2. Reality check — read this before anything else

| # | Issue | Why it matters | Action |
|---|---|---|---|
| R1 | 竞技组 rules name **Spark-HM** and require **M-Robots OS**; you own **Spark-H** | If the organizer enforces the model name, semi-auto entry could be rejected | **Ask the organizer in writing**: is Spark-H + M-Robots OS accepted for 竞技组? Get it in the registration record |
| R2 | Semi-auto stack lives in `/data/local/release` (M-Robots OS release tree); full-auto stack lives in `~/spark_noetic` (Ubuntu 20.04 + ROS Noetic) | Two ROS environments, two `ROS_MASTER_URI`, two catkin spaces on one board — the single biggest technical risk | **M0 audit**: determine whether both trees coexist on one image. If not, build two images + a swap procedure |
| R3 | Source packages are **not** in the material you gave me | Only docs were provided; `auto_match_25.zip`, `semi_auto_match_24`, and the DevEco app project are referenced but absent | Obtain the competition resource pack (zips + DevEco project) before coding starts |
| R4 | Scoring formula "（N+2N-1）" is ambiguous in the PDF (superscript lost) | Changes stacking strategy completely | Confirm per-layer values with judges: likely `N + 2^(N-1)` for 收取区 and `2N-1` for 运输区 |
| R5 | 竞技组 and 自动任务赛 share one map; 分类区 markings are meaningless in 竞技组 | Don't let Squad B's map annotations leak into Squad A's field setup | Separate field-setup checklists per track |

Nothing else in the 15 PDFs contradicts this plan.

---

## 3. System architecture (what we are actually building)

```
┌─────────────────────────┐        ┌──────────────────────────┐
│  HarmonyOS phone app    │  FA    │  spark-HM-PI board       │
│  DevEco 3.1 Beta2       │───────▶│  OpenHarmony 3.1.6       │
│  SDK 3.1.0 (API 9)      │        │  spark_cmd_masker/       │
│  pages: autoMatch,      │        │   spark_cmd_master.c     │
│  autoPage, index,       │        │   spark_arm.c            │
│  tcpTest                │        │  USB-CDC ──▶ NUC         │
└─────────────────────────┘        └──────────────────────────┘
                                              │ /dev/SPARK-HM-PI
                                              ▼
┌────────────────────────────────────────────────────────────────┐
│  NUC — Ubuntu 20.04 / ROS Noetic                               │
│                                                                │
│  SQUAD A (semi-auto)          SQUAD B (full-auto)              │
│  /data/local/release          ~/spark_noetic/src/3rd_app/      │
│  onekey.sh menu 11            spark_hm_task_match/             │
│  semi_auto_match_24/          ├ nodes/ runtask.py              │
│  ├ teleop.py   (WASD)         │        detect_ros.py           │
│  ├ grasp.py    (HSV/YOLO)     │        mark_move.py            │
│  ├ arm_ik.py                  │        cali_node.py            │
│  └ config/                    ├ launch/ run.launch            │
│     grasp_colors.yaml         │         cali.launch            │
│     grasp_heights.yaml        └ config/ cali_config.yaml       │
│     yolo/detect_rknn.yaml              items_config.yaml       │
│                                                                │
│  Shared: spark_slam (gmapping) · spark_navigation (move_base)   │
│          spark_teleop · spark_hm_pi (hm_read_node,             │
│          hm_tcp_client_node :10006 → 副板 OPEN/CLOSE)           │
└────────────────────────────────────────────────────────────────┘
```

**Three layers, three skill sets.** Squad A owns the release-tree layer, Squad B owns the catkin workspace layer, and *both* depend on the board bridge — so the bridge gets a single named owner, not a shared "someone will do it".

---

## 4. Team — 6 people, 3 + 3

Two squads, but **five shared responsibilities** must still have exactly one owner each (a shared responsibility with no owner is how these projects die).

### Squad A — Semi-auto (竞技组)

| ID | Role | Owns | Primary files |
|---|---|---|---|
| **A1** | Squad lead + chassis control | Command mapping phone→board→ROS, driving feel, emergency stop | `teleop.py`, `hm_read_node.cpp` (walk_vel 0.1 / yaw_rate 0.6), app `index` page |
| **A2** | Perception + manipulation | Brick detection, hand-eye calibration, grab/place heights, stacking reliability | `grasp.py`, `arm_ik.py`, `cali*`, `grasp_colors.yaml`, `grasp_heights.yaml`, `detect_rknn.yaml` |
| **A3** | Strategy + match operations | Point math, run order, timing drills, field setup, runbook, judge liaison | Runbook, score sheets, `onekey.sh` procedure |

### Squad B — Full-auto (开源鸿蒙应用赛)

| ID | Role | Owns | Primary files |
|---|---|---|---|
| **B1** | Squad lead + navigation | Mapping, waypoint teaching, move_base tuning, recovery, nav timeout policy | `mark_move.py`, `spark_slam`, `spark_navigation` params, `autoPage`/`autoMatch` pages |
| **B2** | Perception + manipulation | Goods detection, layer detection (2-layer tables), calibration, stacking | `detect_ros.py`, `cali_node.py`, `cali_config.yaml`, `items_config.yaml` |
| **B3** | Orchestration + integration | Task flow, timing budget, board bridge, 副板 OPEN/CLOSE, logging/replay | `runtask.py`, `spark_hm_pi/*`, `run.launch` |

### Shared roles (assign now, one name each)

| Responsibility | Owner | Backup |
|---|---|---|
| Robot custodian (booking, charging, transport, physical condition) | A1 | B1 |
| Board bridge (OpenHarmony firmware + NFC + 副板 TCP) | B3 | A1 |
| Repo/build CI (catkin_make green, image builds, tag policy) | B2 | B1 |
| Data & logs (bag recording, run archive, param history) | A3 | B3 |
| Safety & rules compliance (emergency stop, "no touching the field", early-start rules) | A3 | B3 |
| Judge/organizer communication | A3 | B1 |

### Cadence

- **Daily 15 min standup** per squad (what moved, what's blocked, robot needed?)
- **Weekly 45 min joint sync** — cross-squad decisions only (env conflicts, robot schedule, shared params)
- **One integration owner per phase**, named in the milestone table, not per squad

---

## 5. Work breakdown

### Phase M0 — Environment audit & unblocking *(both squads, blocking)*

Exit criteria:
- [ ] R1 answered by organizer in writing
- [ ] Competition resource pack obtained; `catkin_make` succeeds on a clean workspace
- [ ] Board reachable: `hdc shell ifconfig` shows wlan0 IP; SSH `root`/`123456` works
- [ ] `cd /data/local/release && ./onekey.sh` menu 1 (teleop) and 11 (semi-auto) both launch
- [ ] Decision recorded: **single image** vs **two images + swap** (R2)
- [ ] Phone app project opens in DevEco, signs, installs on the phone
- [ ] One `.bag` recorded of a successful manual teleop run (baseline)

### Phase M1 — Walk before run (parallel, both squads)

| Squad A | Squad B |
|---|---|
| Reproduce menu 11 semi-auto match end-to-end | Reproduce `run.launch`, drive with the app, build a map with `open_guide:=true` |
| Record baseline: bricks/minute, grab success rate | Teach `wp_1..wp_4`, `t_1`, `t_2`, `warehouse_l/m`, save `mark_map.pkl` |
| Emergency stop verified (single key) | Verify `/hm_task_gate_cmd` → 副板 shows OPEN then CLOSE |

Exit: both squads can complete a *scored dry run* of their task with default parameters.

### Phase M2 — Reliability (the phase that wins matches)

Squad A: calibration reproducibility (the 3-value spread check), HSV re-tuning under real lighting, suction failure recovery, stacking on layer 2/3, driving precision.
Squad B: nav success rate over 10 identical runs, recovery behaviour when a goal aborts, detection recall on clock/doll/cup, 2-layer depth logic, place-position accuracy for stacking.

Metric gates (must be measured, not felt):
- Semi: grab success ≥ 85 % over 30 attempts; average brick cycle time recorded
- Auto: nav success ≥ 90 % over 10 runs; misplacement rate ≤ 10 %

### Phase M3 — Strategy & optimisation

Squad A: point-optimal run order (blue water brick first in every zone that will hold veg/red), stacking depth vs. risk, 4-minute route plan.
Squad B: waypoint A–D before any grab (8 pts), cup-first placement (2N+1 is the best per layer), OPEN/CLOSE timing, 15-minute budget allocation.

### Phase M4 — Integration on one robot

- Build the dual-stack image / swap procedure; time it
- Full dress rehearsals: app→board→ROS→arm, both tracks, back-to-back
- Failure drills: nav abort, lost detection, arm unreachable pose, board disconnect, Wi-Fi loss

### Phase M5 — Match readiness

- Printed runbooks (one page per track), laminated
- Spare parts, charging plan, transport checklist
- Two full rehearsals with an outside "judge" calling violations
- Freeze code

---

## 6. One robot, six people — contention plan

This is the constraint that decides whether the plan works.

1. **Robot slots, not "whoever asks".** Fixed 2-hour blocks in a shared calendar a week ahead. A squad may not hold the robot for a whole day.
2. **Simulation and dry-desk first.** Both squads must be able to develop with no robot:
   - Squad A: ROS bag playback of `/camera/color/image_raw` to tune HSV/YOLO thresholds.
   - Squad B: `move_base` in a saved map with a fake `cmd_vel`, plus static images for detection tuning.
   - Board: one spare spark-HM-PI (or the VM) for firmware/NFC work, so flashing never consumes robot time.
3. **Hardware parity.** At least two dev PCs with DevEco + VirtualBox images so the phone app and board firmware can be built without the robot.
4. **Swap protocol (R2).** Whichever way M0 resolves, write it as a numbered procedure with a rollback step. Time it. If it takes longer than the 4-minute prep window, it must happen before the robot is placed in the field.
5. **Freeze window.** No parameter changes in the final period before a match; all tuning is logged with a revert value.
6. **One driver per run.** Only the named operator touches the keyboard during a timed rehearsal — mirroring the rule that only the on-field player may touch the computer.

---

## 7. Engineering practices

**Repo layout**

```
spark-competition/
├─ README.md                  # who owns what, how to build, how to swap stacks
├─ dispatch/                  # board firmware (OpenHarmony) + flashing notes
├─ semi_auto/                 # Squad A: teleop.py, grasp.py, arm_ik.py, config/
├─ full_auto/                 # Squad B: catkin ws overlay, nodes, launch, config/
├─ app/                       # DevEco project (HarmonyOS phone app)
├─ tools/                     # bag record/play, param diff, score calculator
├─ docs/                      # runbooks, calibration records, field maps
└─ logs/                      # dated run folders (bag + score + params snapshot)
```

**Rules**
- Every tuning change is a commit with the measured before/after in the message.
- Every scored run produces `logs/YYYY-MM-DD_HHMM_<track>/` containing: bag, params snapshot, score sheet, one-line result.
- `config/` values are the only sanctioned knobs; no magic numbers in code without a comment naming the file that overrides it.
- Board firmware and app are versioned together — the app's `controlDta` code table is a **contract**; changing a code means changing `spark_cmd_master.c` in the same PR.

**Param reference (from the docs — verify against your actual build)**

| Item | Path | Default |
|---|---|---|
| Blue brick HSV | `semi_auto_match_24/config/grasp_colors.yaml` | lower `[95,90,80]`, upper `[130,255,255]` |
| Arm grab heights | `.../config/grasp_heights.yaml` | `z_up: 20`, `z_down: -45` |
| Chassis speeds | `.../usr/lib/semi_auto_match_24/teleop.py` | `walk_vel 0.1`, `run_vel 0.5`, `yaw_rate 0.5`, `yaw_rate_run 1.0` |
| Detection thresholds | `.../config/yolo/detect_rknn.yaml` | `conf_thres 0.25`, `nms_thres 0.45`, `max_det 100` |
| Hand-eye calibration | `/data/local/release/usr/thefile.txt` | regenerated by `onekey.sh` menu 7 |
| Auto calibration | `spark_hm_task_match/config/cali_config.yaml` | per `z=32`: `k1,b1,k2,b2` |
| Goods→table map | `spark_hm_task_match/config/items_config.yaml` | `warehouse_l`, `warehouse_m` |

---

## 8. Scoring strategy

### Semi-auto (4 min, 竞技组)

Hard constraint from the rules: a zone with veg/red bricks **must** contain a blue water brick, or those bricks score zero (blue still counts alone). Therefore the run order is not "nearest brick first" — it is **blue first, per zone, then everything else**.

- Keep every brick's *contact face parallel to the ground* (non-parallel = layer 1 only).
- Never place red before all blue+veg are in scoring zones.
- Never push a brick with the arm (< 10 cm, arm sideways) — it scores nothing and earns a warning.
- Never rotate the end suction cup by program (−5).
- Confirm the layer formula (R4) before committing to a stacking plan above layer 2.

### Full-auto (15 min, 开源鸿蒙应用赛)

Free/cheap points first, in this order:
1. **Waypoints A, B, C, D before any grab** — 2 each, once only, 8 points, pure navigation.
2. App-triggered movement +1, board green START +1, first OPEN +2, final CLOSE +2 → 6 points that depend on the bridge working, so the bridge is a scoring component, not plumbing.
3. Goods: cup `2N+1`, doll `2N`, clock `2N-1` — **cups are worth the most per layer**, so fill the cup table deepest.
4. Keep the standalone board showing red √ before start (−2 if missing).
5. Budget: 4 waypoints + 12 goods in 15 min ⇒ roughly 50 s per goods cycle including nav. Track this from M1 onward, not at M4.

---

## 9. Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Spark-H not accepted for 竞技组 | Medium | Fatal for one track | R1 written confirmation in M0; fallback = full-auto only |
| Two ROS stacks cannot coexist | Medium | Blocks integration late | M0 decision + timed swap procedure + rollback |
| Source pack late / differs from docs | Medium | Slippage | Start tools/runbooks that don't need code; ask for the pack on day 1 |
| Robot contention | High | Both squads slow | Slot calendar + sim-first rule (Section 6) |
| Calibration drifts (lighting) | High | Missed grabs | Calibration before every session; record values; the 3-value spread check as a gate |
| Nav timeout / localization drift | High | Lost run | 60 s abort policy already in code; add recovery + re-localize step |
| Detection misses in real light | High | Empty picks | Bag replay tuning; conf_thres tuning with `debug_image` evidence, not guesses |
| Bridge firmware regression | Medium | 6 points lost + app dead | Code-table contract + versioned flashing procedure + spare board |
| Single robot failure | Low | Fatal | Spares list, pre-match self-test checklist |

---

## 10. Definition of done

**Squad A**
- 3 consecutive clean runs at target score with zero rule violations
- Emergency stop demonstrated
- Runbook fits one page; a teammate who didn't write it can operate from it

**Squad B**
- 3 consecutive full runs: A–D visited, ≥ 8 goods placed correctly, OPEN→CLOSE sequence correct, no manual intervention
- App-only start path verified (no SSH needed after start)
- Nav failure degrades gracefully (skips item, continues, still finishes)

**Both**
- Stack swap documented, timed, and rehearsed twice
- `logs/` contains the full rehearsal archive

---

## 11. First 7 days (concrete)

| Day | Action | Owner |
|---|---|---|
| 1 | Send R1 question to organizers; request resource pack (zips + DevEco project) | A3 |
| 1 | Book robot slots for the next 2 weeks in the shared calendar | A1 |
| 2 | Repo created with the Section 7 layout; everyone commits a "hello" file | B2 |
| 2–3 | M0 audit: SSH in, `hdc shell ifconfig`, `onekey.sh`, run menu 1 and menu 11, record every IP/port/version | A1, B3 |
| 3–5 | Squad B: `catkin_make` clean workspace, `run.launch`, app-driven teleop, first map | B1, B3 |
| 3–5 | Squad A: reproduce semi-auto match, 30-grab baseline, record success rate | A1, A2 |
| 5 | Decide single-image vs dual-image (R2); write swap procedure v0 | A1, B1 |
| 6 | Joint sync: publish baselines, agree metric targets, freeze the code-table contract | All |
| 7 | Desk work only — sim/replay tuning, runbook v0, score calculator | All |

---

## 12. Decisions I need from you

1. Does the Spark-H ship with the M-Robots OS release tree (`/data/local/release`) present, or only Ubuntu + `~/spark_noetic`?
2. Can you get `auto_match_25.zip` / `semi_auto_match_24` / the DevEco app project?
3. Two images or one shared image — your preference, given the swap-time cost?
4. Do you already have a spare spark-HM-PI board for firmware work?
5. Match date and which track is scored first (that sets which squad gets the robot in the final window).

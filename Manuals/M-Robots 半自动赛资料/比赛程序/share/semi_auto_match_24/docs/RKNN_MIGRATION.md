# 2024 半自动任务赛程序 · OpenHarmony 交叉编译适配说明

> 目标板：KaiHong RK3588 / OpenHarmony（musl + OHOS ABI）
> 工具链：`spark-cross`（chroot + catkin_tools + DESTDIR 装到 `install/usr`）
> 本次范围：**交叉编译适配 + 把模型推理换成 RK3588 NPU 上的 YOLOv5**。
> 板端真机验证另做，见文末清单。

---

## 1. 原程序与适配后的对应关系

原 zip：`spark_noetic/src/3rd_app/semi_auto_match_24-master.zip`
（`semi_auto_match_24-master/semi_auto_match_24/` + 根目录一个 `LocationCentre.msg`）

| 原文件 | 适配后 | 说明 |
|---|---|---|
| `scripts/detect.py` | **删除**，换成 `nodes/detect_rknn.py` | TensorFlow SSD MobileNet → RKNN YOLOv5 |
| `scripts/grasp.py` | `nodes/grasp.py` | 抓放算法不变，只修板上会挂的地方 |
| `scripts/teleop.py` | `nodes/teleop.py` | 按键映射不变 |
| `LocationCentre.msg`（zip 根目录） | `msg/LocationCentre.msg` | 收进包里，成为本包自己的消息 |
| `CMakeLists.txt`（全注释） | 重写 | 原来一行 install 都没有 → 编了也没产物 |
| `package.xml`（无依赖） | 重写 | 依赖真源，见 §3 |
| `launch/semi_auto.launch` | 重写 | 去掉 xterm、修 rviz、teleop 改手动 |
| `rviz/*.rviz` | 原样保留 + 新增 `semi_auto.rviz` | 原来指向 skiplist 包里的布局 |
| — | `nodes/rknn_yolo.py`、`python/rknn/` | 复用 2026 版已验证过的 NPU 侧代码 |
| — | `nodes/roi_map.py` | ROI ↔ 整幅图像坐标换算，单独可测 |
| — | `config/yolo/`、`launch/detect_rknn.launch` | 检测参数与标签表 |
| — | `test/test_rknn_yolo_postprocess.py` | 解码/NMS/坐标契约回归测试 |

原 zip 里的 `scripts/.idea/`、`scripts/.vscode/`（IDE 配置）没有搬过来。

---

## 2. 推理：TensorFlow SSD → RKNN YOLOv5

**为什么必须换**：板上装不了 TensorFlow，`object_detection` / `mscoco_label_map.pbtxt` /
`frozen_inference_graph.pb` 都不在镜像里；而 RK3588 有 NPU，厂商随镜像提供了
`librknnrt.so` 与官方 `rknn_model_zoo` 的默认 YOLOv5 模型。

**怎么换**：

* 推理运行时 = `librknnrt.so` 的 ctypes 兼容层（`python/rknn/api.py`）。
  板上是 musl，PyPI 的 `rknn-toolkit-lite2` 只有 glibc 轮子，pip 装不上；
  C ABI 稳定，用 ctypes 直连是唯一可行路径。
* YOLOv5 解码 + NMS = `nodes/rknn_yolo.py`（v5 三尺度头、anchor、先过滤后解码、按类 NMS）。
* 模型 = 官方默认 `yolov5s-640-640.rknn`（COCO 80 类），
  路径 `/data/local/release/rknn-demo/model/yolov5s-640-640.rknn`，
  可用 `~model_path` 覆盖（`config/yolo/detect_rknn.yaml` 或 `model_path:=...`）。

**对外接口一字未改**，所以 `grasp.py` 与 launch 都不用跟着动：

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `image` | `sensor_msgs/Image`（launch 里 remap 到相机话题） |
| 发布 | `location` | `semi_auto_match_24/LocationCentre` |
| 发布 | `debug_image` | `sensor_msgs/Image` |

---

## 3. 两处**必须**说清楚的语义契约

这两处错了都不会报错，只会"抓不到"或"抓偏"，所以单独写出来，并各有一条回归测试。

### 3.1 类别 id：西兰花必须还是 56

`grasp.py` 里判断西兰花是 `loc.obj_id == 56`。这个 56 是
`tensorflow_object_detector` 那份 `mscoco_label_map.pbtxt` 的 **90 类编号**
（中间有 12 个空洞）。而 YOLOv5 输出的是 **COCO-80 下标**，西兰花是 **50**。

直接发模型下标 → `grasp.py` 的西兰花分支永远不触发，且没有任何报错。

处理：`config/yolo/coco_labels.txt` 是三列格式 `模型下标 外部id 名字`，
`detect_rknn.py` 默认（`publish_tf_ids: true`）按它把 50 换算成 56 再发。
`test/test_rknn_yolo_postprocess.py::test_grasp_id_contract` 钉住这条契约。

> 换模型/换类别表时注意：映射表必须是 80 行、下标连续。

### 3.2 坐标：ROI 内坐标 → 整幅图像坐标

原程序只把 640×480 里的 `image[160:480, 160:480]`（320×320）送进模型，
再把框中心换算回**整幅图像坐标**发出去 —— 因为机械臂标定文件 `thefile.txt`
是整幅图像坐标系下的线性映射。

原版写法与本次实现的差别：

| | 原 `detect.py` | 本次实现（默认） |
|---|---|---|
| x 偏移 | `0.25 * 640 = 160` | `roi` 左上角 = 160 |
| y 偏移 | `0.33 * 480 = 158.4` | `roi` 左上角 = 160 |

**x 方向逐像素一致；y 方向默认比原版大 1.6 px。** 原版的 0.33×480 显然是想表达
裁剪原点 160，只是用了近似系数。要**逐像素复现原版**（例如沿用旧的标定文件），
把参数设成：

```yaml
roi: [160, 160, 320, 320]
roi_offset: [160, 158.4]     # 留空 [] 则用 roi 左上角 (160, 160)
```

`test_roi_mapping_matches_original` 同时钉住了"x 一致"、"默认 y 差 1.6 px"、
"显式 158.4 时 y 也一致"三件事。

---

## 4. 板端会直接挂掉、必须改的地方

| 症状 | 原因 | 处理 |
|---|---|---|
| 节点 import 就死 | `from vision_msgs.msg import ... LocationCentre` —— `vision_msgs` 里没有这个消息（它在原 zip 根目录，属于本包） | 改 `from semi_auto_match_24.msg import LocationCentre` |
| import 不到机械臂消息 | 原文件 `from swiftpro.msg import *`，而 `swiftpro` 在 `build.sh` 的 skiplist 里 | 优先 `spark_carry_object.msg`（本工具链会编），取不到才退回 `swiftpro`。两者 `position`/`status` 字段定义逐字节一致 |
| teleop 起不来 | `launch-prefix="xterm -e"`，板上没有终端模拟器 | 去掉前缀，并默认**不自动拉起**（交互式节点要真 tty，请另开窗口 `run rosrun semi_auto_match_24 teleop.py`） |
| rviz 起不来 | 原布局来自 `tensorflow_object_detector`（skiplist 包，板上没有） | 用本包自带的 `rviz/semi_auto.rviz`；rviz 加 `launch-prefix="run_gui"` 拿 GL 上下文 |
| **rviz 3D 画面空白/发白，什么都不画** | 布局的 **Fixed Frame** 是不是本 launch 真有的帧 | 见 §4.1：抄 SLAM 布局会把 Fixed Frame 抄成 `map`，而本 launch 不起 SLAM |
| grasp 节点抛异常 | `cv2.imshow` 无条件调用，板上没有 GL 上下文 | 由 `~show_debug` 控制（默认关）；开启时 launch 自动套 `run_gui`；画图失败只警告一次 |
| 轮廓为空时崩 | `cv2.drawContours(img, [], 0, ...)` 断言失败 | 加空判断 |
| numpy 2.x 下崩 | `np.int0` 已被移除 | 换等价的 `np.intp` |
| teleop 静默失效 | 模块顶层就建 `rospy.Publisher`，而 `init_node` 在函数里才调 | Publisher 挪到 `init_node` 之后；并加 `isatty()` 前置检查，不是终端就明确报错 |
| 编了没产物 | 原 `CMakeLists.txt` 的 install 段全被注释 | 重写：消息生成 + `catkin_install_python` + `install(DIRECTORY ...)` |

---

### 4.1 rviz 的 Fixed Frame：别抄 SLAM 布局

**症状**：VNC 里 rviz 起来了、OpenGL 也正常（`OpenGL device: Mali-G610 (Panfrost)`），
但 3D 画面空白 / 模型发白，什么都画不出来。

**根因**：rviz 的 **Fixed Frame** 必须是一个**本 launch 真的会发布**的 TF 帧，
否则它在状态栏报 `Fixed Frame [xxx] does not exist`，然后整块 3D 视图什么都不画。

本 launch 实际存在的帧（板端 `rostopic info /tf` + `rostopic echo /tf` 实测）：

| 帧 | 谁发 | 本 launch 有没有 |
|---|---|---|
| `odom` | `spark_base_server`（发 `odom → base_footprint`） | **有** |
| `base_link` | `robot_state_publisher` | 有 |
| `camera_link` / `camera_*` | `realsense2_camera`（静态） | 有 |
| `laser` | `robot_state_publisher`（URDF 里的固定关节） | 有 |
| **`map`** | SLAM / `map_server` | **没有** —— 本 launch 不起 SLAM |

一开始我直接抄了 `spark_gmapping_d435.rviz`（它是**建图**布局，Fixed Frame = `map`），
所以上板就白屏。`auto_match.rviz`（2026 版）也是 `map` 中心，但**它的 launch 会起
SLAM 或 map_server**，所以那份没问题、抄过来就有问题。

**现在的做法**：`rviz/semi_auto.rviz` 以官方的
`spark_description/rviz/urdf.rviz` 为底 —— 那份是"看机器人"布局，Fixed Frame 就是
`odom`，且恰好配对 `joint_state_publisher` + `robot_state_publisher` 这一对节点；
再把 PointCloud2 面板换成雷达 + 两路图像（LaserScan/Image 的字段块直接从
`auto_match.rviz` 整块搬，同板同 rviz 版本，schema 一定对）：

```
Grid / RobotModel / TF(关) / LaserScan(/scan)
/ Image(/camera/color/image_raw) / Image(/debug_image)      Fixed Frame: odom
```

**视角与窗口也一起调了**（官方 `urdf.rviz` 存的是 Distance 6.64 m、窗口 1859×1056）：

| 项 | 原值 | 改成 | 为什么 |
|---|---|---|---|
| `Target Frame` | `<Fixed Frame>`（= odom 原点） | `base_link` | 相机跟着机器人走；盯 odom 原点的话，车一开走就不在画面里了 |
| `Distance` | 6.64 m | 2.5 m | 底盘只有 34 cm，6.6 m 外只剩一个小点 |
| 窗口尺寸 | 1859×1056 | 1100×650 | 板上 Xvfb 就是 1100×650；尺寸不匹配时 Qt 按旧布局摆面板，机器人会被挤到画面角落 |

生成脚本 `test/gen_rviz.py` 一次搞定，要改帧或视角直接改脚本里的 `VIEW_FIXES` 重跑，
或者在 rviz 里 `Global Options → Fixed Frame` 改完 `File → Save Config As` 覆盖。

---

## 5. 刻意**没有**改的行为

原程序里的这些写法看着奇怪，但它们直接影响标定基准或比赛策略，改动属于另一件事，
本次一律保留（只在代码注释里标出来）：

* `image_cb` 与 `image_cb_red` 订阅**同一个话题**、每帧各跑一遍 HSV（浪费，但改了会动时序）。
* 距离度量 `sqrt((x_mid-320)**2 + (480-y_mid)**2)` 按 640×480 写死。
* `grasp(self, y, x, layer)` 的形参顺序与实参顺序不一致（调用处传的是 `(x, y)`）——
  保持原样，改了会改变机械臂落点。
* `broccoli_found_count >= 10` / `water_found_count >= 30` 等稳定性阈值。

### 5.1 改过的一处真 bug：水源与红块**共用计数器**

原 `grasp.py` 里 `image_cb_red` 和 `image_cb` **共用** `self.water_found_count` /
`self.found_water`。两个回调订阅同一个话题、每帧各跑一次，于是互相清零：

```
image_cb     （蓝）：中心稳定 -> water_found_count += 1
image_cb_red （红）：没找到轮廓 -> water_found_count = 0     ← 把蓝攒的清了
```

**现象**：按 G 抓水源，只看到 `[WARN] finding water` 一直刷、机械臂不动 ——
`found_water` 永远到不了 True。

**实测（2026-10-14，板端探针）**：60 帧里蓝色轮廓 **0/60**（画面是灰地毯 + 一块
白方块，S≈9），红色轮廓 59/60 但中心每帧跳几十像素（不稳定）→ 共享计数恒为 0。

**处置**：红色用自己的 `red_found_count` / `found_red`，`grasp_cp` 里抓红块那条
分支（`'6'` / `'12'`）等 `found_red`。

> 注意：这次同时暴露了"**有红没蓝**时计数必为 0"。即使把蓝色水源放进视野，
> 只要红分支还在清零，照样抓不到 —— 所以这是必修项，不是优化。

### 5.2 水源认不出来时的排查顺序

`grasp.py` 认水源靠的是**固定 HSV 区间**：`H[95,130] S[90,255] V[80,255]`。

```sh
# 板上量一量相机到底看到什么（只订阅，不改任何东西）
run python3 /data/local/release/usr/share/semi_auto_match_24/test/probe_grasp_colors.py 60
```

它照抄 grasp.py 的两条流水线，打印：蓝/红各有几帧出轮廓、中心稳不稳、
按现在的共享计数 vs 各用各的计数分别会不会 `found_water`，并把当前帧与两张掩膜
落到 `/data/local/tmp/probe_*.png`。

| 观察 | 结论 |
|---|---|
| 蓝轮廓 0/N，且全图 S 均值很低 | **画面里没有蓝色目标** —— 把水源放进相机视野（相机装在机械臂上，朝下看夹爪附近） |
| 蓝轮廓有，但中心每帧跳 >4px | 目标在动 / 光斑不稳 —— 拿稳，或调阈值 |
| 有蓝有轮廓但 S 明显 <90 | 目标不是饱和蓝（白 / 浅蓝 / 过曝）—— 换目标，或放宽 `LowerBlue` 的 S 下限 |

### 5.3 抓取高度：一个高度给三种目标，并且能现场调

**单位是毫米** —— `swiftpro` 的 `position.z` 直接拼成 Gcode `G0 X.. Y.. Z..`，
`swiftpro_write_node.cpp` 的注释写明 `x,y,z(mm)`，Z=0 是机械臂底座平面。

原 `grasp()` 是按**按键大小写**选高度，和抓什么东西无关：

```python
if layer in ['10', '11', '12']:   # 大写 G/T/E
    z_up, z_down = 90, 30
else:                             # 小写 g/t/e
    z_up, z_down = 20, -55
```

**两支差了 85mm** —— 这才是"三个目标高度看着不一样"的真正原因。

板端日志实测（`swiftpro_write_node` 每次都会 `ROS_INFO` 打 Gcode，
所以实际发出去的 Z 是确定的，不用猜）：

| 你按的键 | 走的支 | 实际发的 Gcode | 结果 |
|---|---|---|---|
| 小写 `g` / `t`（水源、蔬菜） | `else` | `Z20` → `Z-55` | 太低，机械臂顶到自己 |
| 大写 `E`（红色方块） | `if` | `Z90` → `Z30` | 太高，吸盘够不到 |

**现在统一成一套，大小写只决定发哪条指令、不影响高度**：

| 项 | 值 | 说明 |
|---|---|---|
| `z_up` | 20 | 悬停高度 |
| `z_down` | **-45** | 原小写分支是 -55，实测太深，抬高 1cm |

**现场调（不用重新编译）**：

```sh
# 板上直接改，然后重启 grasp（或重跑菜单第 11 项）
vi /data/local/release/usr/share/semi_auto_match_24/config/grasp_heights.yaml
#   顶到自己/压住物体 -> z_down 调大；吸盘够不到 -> z_down 调小
```

单跑时也能临时覆盖：`run rosrun semi_auto_match_24 grasp.py _z_down:=-40`。
节点启动时会打印实际用的高度（`抓取高度(mm): z_up=.. z_down=..`），
抓取时还会打一条 `抓取目标=... layer=... -> z_up=.. z_down=..`，便于核对。

> **怎么自查实际发了什么 Z**：`swiftpro_write_node` 把每条 Gcode 都打进
> `<log>/<run>/swiftpro_write_node-9-stdout.log`，直接
> `grep -o 'G0 X.*Z[-0-9.]*'` 就能看到真实轨迹 —— 比看机械臂动作可靠得多。

### 5.4 "够不到"的位姿：机械臂不动，但泵照开、臂照抬

**症状**：抓某个目标时机械臂**不动**，却看到吸泵先开、然后直接抬到高位；
`swiftpro_rviz_node` 刷 `[ERROR] Inverse kinematic is wrong`。

**这不是两个问题，是同一个**：标定把像素映射成机械臂坐标，但机械臂
**够不到所有位置**。够不到时固件不接受那个位姿（机械臂原地不动），
而 `grasp()` 后面照样 `pub2.publish(1)` 开泵、照样抬到 `(230, 0, 160)` ——
现场看起来就是"没抓就跑了"。

那句 `Inverse kinematic is wrong` 来自 `swiftpro_rviz_node`，
**那个节点只把位姿画到 rviz、不参与实际运动**：

```cpp
if ( swiftpro_ik(position, angle) )   // 算不出来就只报一句
    all_joints_state(angle);
else
    ROS_ERROR("Inverse kinematic is wrong");
```

所以它是**同一个数学结论的旁证**，不是原因 —— 但它恰好是最早能看到的那条线索。

**可达区域（实测，640×480，标定用板上 thefile.txt，z=20 与 -45 都要可达）**：

```
行   0 ~ 110   整行都够不到        pos.x > ~370mm，超出臂展
行 112 ~ 300   只有中间一条横带可达，越往下越宽
行 320 ~ 400   整行都可达   ← 物体放这里最稳
行 410 ~ 480   底部中央出现缺口    列约 280~440，离底座太近折不过来
```

```sh
# 打印这张图
python3 test/reachable_area.py
# 板上跑：抓一帧相机，把可达区叠成绿/红存 PNG
run python3 /data/local/release/usr/share/semi_auto_match_24/test/reachable_area.py --overlay
```

**现在的行为**：`grasp()` 在发位姿前先判可达（`nodes/arm_ik.py`，
数学照抄 `swiftpro_rviz_node.cpp` 的 `swiftpro_ik()`）；够不到就
**直接放弃本次抓取（不吸、不抬）** 并打一条说清该往哪挪的 ERROR。

> 数学放在 `nodes/arm_ik.py` 一份，`grasp.py` 与 `test/reachable_area.py` 共用 ——
> 两处各写一份的话，判据会漂移，那就比不判还糟。

### 5.5 红色方块检不到：HSV 的红色是**两端**的

**症状**：物块明明在可达区（`reachable_area.py --overlay` 里是绿的），就是抓不到；
抓水源/蔬菜却正常。

**根因**：红色在 OpenCV 色环上是**两端** —— H≈0 **和** H≈179。
原程序写的是：

```python
LowerRed = np.array([146, 80, 46])     # 只覆盖**高端**
UpperRed = np.array([180, 255, 255])
```

而板上这块方块实测 **H≈0~5**（直方图峰值在 H=1~3）。**一个像素都匹配不到**，
于是检测器只能去锁画面里的噪声，算出来的抓取坐标全是垃圾。

**还有第二个坑**（在真实帧上跑完整流水线才暴露）：

| 方案 | 结果 |
|---|---|
| 原样 `H[146,180]` + H 通道阈值 20 | 无轮廓 |
| 只把区间改成 `H[0,15]`，阈值仍是 20 | inRange 命中 **19238** 像素，**仍然无轮廓** |
| `H[0,15]` + 阈值降到 1 | 轮廓 22 个，最大面积 13788，中心 (328, 299) |
| **`H[0,15]` + 直接对掩膜二值化** | **轮廓 1 个**，面积 14998，中心 (329, 297) ← 采用 |
| `H[0,15] ∪ [160,180]` | 最大轮廓包围盒横跨半个画面 ← **把左侧鞋子/杂物也当成红色** |

原因：红色分支那步 `cv2.threshold(blurred, 20, ...)` 是对**取出来的 H 通道**取阈值，
`H≈0~5` 会被 20 直接抹平 —— 区间改对了也没用。改成直接对掩膜二值化，
顺带避开"对环绕的 H 做 blur"在红端产生的伪影。

**处置**：区间改成 `H[0, 15]`（**不要**把 `[160,180]` 并进来），
阈值走掩膜；两个区间可由 `config/grasp_colors.yaml` 覆盖。
改完中心落在 (329, 297) → 机械臂 (186.2, 21.2)、半径 187 → **可达**。

> 换方块颜色/材质后重新量：`test/probe_grasp_colors.py` 会打印高饱和像素的
> 色相直方图；`H` 落在哪就照那个改 `grasp_colors.yaml`。

唯一一处**必须改**的默认值：手眼标定文件的查找顺序。原程序写死 `$HOME/thefile.txt`，
而本板 `HOME=/`，实测标定文件在 `/data/local/release/usr/thefile.txt`。
现在按 `~calib_file` > `$SEMI_AUTO_MATCH_CALIB` > `$HOME/thefile.txt` >
`/data/local/release/usr/thefile.txt` 找第一个存在的，并把选中的路径打进日志
（标定文件选错不会报错，只会让机械臂抓偏）。

---

## 6. 编译

```bash
# 推荐：增量编译（SDK 只解一次，改几个文件重编几十秒）
wsl -d Ubuntu-22.04 -u root -- bash spark-cross/board/build_persist.sh

# 或者用原始入口（每次全新临时目录，解 SDK 约 8 分钟）
cd ~/spark/spark-cross && sudo ./demo.sh 2>&1 | tee /tmp/build.log
```

产物落点：

```
install/usr/lib/semi_auto_match_24/{detect_rknn.py,grasp.py,teleop.py,rknn_yolo.py,roi_map.py}
install/usr/share/semi_auto_match_24/{launch,config/yolo,rviz,docs,python/rknn,msg,package.xml}
install/usr/lib/python3.12/site-packages/semi_auto_match_24/msg/   # LocationCentre
install/usr/include/semi_auto_match_24/LocationCentre.h
```

**注意 `install/` 是累积的**：判断"这次到底编出来没有"要看构建日志的 Summary，
不是看目录里有没有东西。

> 已知的构建脆弱点（与本次改动无关）：`build.sh` 用 `--parallel-packages` 并行跑
> 各包的 `make DESTDIR=... install`，而每个包都会往 `$DESTDIR/usr/` 写同一批
> catkin setup 文件（`setup.bash` / `env.sh` / `_setup_util.py` …）。并发时会撞出
> `file INSTALL cannot set modification time ...: No such file or directory`，
> 表现为某个包 Failed、其余全部 Abandoned。**重跑一次即可**（本次就复现过一次）。

---

## 7. 自测

### 7.1 契约回归（任何机器，纯标准库）

```bash
python3 spark_noetic/src/3rd_app/semi_auto_match_24/test/test_rknn_yolo_postprocess.py
```

钉住本赛项最容易静默出错的两条契约：**西兰花对外 id 必须是 56**
（YOLOv5 给的是 50）、**ROI 坐标换算必须与原 `detect.py` 一致**
（x 逐像素一致、y 默认差 1.6 px、显式 `[160,158.4]` 时也一致）。

### 7.2 解码回归（需要 numpy + cv2 → 板上跑）

同一个脚本的第二层，覆盖 YOLOv5 解码 vs `rknn_model_zoo` 参考实现逐框一致、
NMS 真在抑制重叠框、letterbox 正反闭环、标签文件三种写法、缺 anchors 时明确报错。

```sh
# 板上
run python3 test/test_rknn_yolo_postprocess.py
```

> 开发机上跑不了这一层：SDK 的 target sysroot 里只有 **aarch64** 的 cv2
> （x86_64 的宿主 python 加载不了），而且 **numpy 根本不在 SDK 里**
> —— 它是板端运行时依赖。脚本会自动 `[SKIP]` 第二层并以 0 退出。

### 7.3 launch 静态引用检查（任何机器）

```bash
python3 spark_noetic/src/3rd_app/semi_auto_match_24/test/check_launch_refs.py
```

查 `$(arg)` 有没有出处、`$(find PKG)` / `<node pkg=...>` 指向的包在不在
（工作区 ∪ SDK 预装），以及有没有引用到 **skiplist 里的包** ——
`cannot launch node of type` 就是这么来的，交叉编译阶段完全看不出来。
（`roslaunch --nodes` 在本机跑不了：SDK 里缺 `defusedxml`。）

---

## 8. 部署到板子

一条命令（三处落点一起推、板端先备份、装一键菜单、逐文件 md5 校验）：

```bash
printf '123456\n' > /tmp/.board_pass && chmod 600 /tmp/.board_pass   # 只需一次
bash spark-cross/board/deploy_semi_auto_match.sh
```

回滚：`cd /data/local/release && tar xzf /data/local/tmp/backup-semi_auto_match_24-<时间戳>.tgz`

要点：

* **三处落点缺一不可**：`usr/lib/<pkg>`（节点与被 import 的模块）、
  `usr/share/<pkg>`（launch/config/python/rknn/package.xml）、
  `usr/lib/python3.12/site-packages/<pkg>`（消息包）。少一处就是
  `ModuleNotFoundError` 或 `cannot launch node of type`，而编译期完全看不出来。
* 一键菜单 `/data/local/release/onekey.sh` 是**两个赛项共用**的共享文件，
  部署脚本会覆盖它（先按时间戳备份），菜单里第 11 项就是本赛项。
* 本包依赖 `spark_carry_object` 的消息（`grasp.py` 的机械臂指令），板端已装；
  若某块板只有 prebuilt 的 `swiftpro`，代码会自动退回它，不用改。
* 模型与运行时是厂商件，随镜像部署：
  `/data/local/release/rknn-demo/{model/yolov5s-640-640.rknn,lib/librknnrt.so}`。
  路径不对时节点会 `logfatal` 并退出（不会静默发空消息）。
* 手眼标定文件在 `/data/local/release/usr/thefile.txt`（板端 `HOME=/`，
  原程序那句 `$HOME/thefile.txt` 落不到这儿，见 §5）。
* 板端启动一律走 `run` 包装（裸 `python3` 缺 `libpython3.12.so`）：

  ```sh
  run roslaunch semi_auto_match_24 semi_auto.launch
  ```

* 板端 `/tmp` 是**只读**的：任何带 heredoc / `sed -i` 的脚本都要先
  `export TMPDIR=/data/local/tmp`，否则报
  `can't create temporary file /tmp/…: Read-only file system`。

---

## 9. 板端验证

### 9.1 已做（2026-10-14，板 192.168.100.118）

部署：`bash spark-cross/board/deploy_semi_auto_match.sh`（三处落点一起推，
30 个文件逐个 `md5sum -c` 通过；一键菜单一并更新并备份）。

| 验的东西 | 结果 |
|---|---|
| 交叉编译 | `build_persist.sh` 全绿，43/43 包成功 |
| 包能不能被找到 | `run rospack find semi_auto_match_24` → `/data/local/release/usr/share/semi_auto_match_24` |
| 消息能不能 import | `import semi_auto_match_24.msg` → `LocationCentre` OK |
| launch 能不能解析 | `roslaunch --nodes semi_auto.launch` → 12 个节点全部解析（含 realsense / swiftpro / rviz / ydlidar） |
| 模型 ↔ 运行时 | `api=1.5.0 / driver=0.9.5 / head=v5 / nc=80` —— 同代，能加载 |
| NPU 真推理 | 640×640 输入：**NPU 17–19 ms + 后处理 0.9 ms ≈ 19–21 ms/帧** |
| 检测节点真起来 | `rosnode list` → `/detect_rknn`；发布 `/location [semi_auto_match_24/LocationCentre]` + `/debug_image [sensor_msgs/Image]`；订阅 remap 到 `/camera/color/image_rect_color` |
| 节点启动日志 | `roi=(160,160,320,320) roi_offset=(160.0,160.0)`、`类别 id 已按映射表换算（80 类）` |
| 契约回归（两层） | 板上 `ALL PASS` —— 含开发机上跑不了解码层 |
| 依赖 | 板上 `numpy 1.26.4 / cv2 4.11.0 / rospy / cv_bridge / rospkg` 齐 |
| 手眼标定文件 | `/data/local/release/usr/thefile.txt` 在（`HOME=/`，原程序那句 `$HOME/thefile.txt` 找不到东西） |
| **rviz 真的画出来了** | 截屏取证：`Global Status: Ok` / `Fixed Frame: OK`，机器人模型（带贴图）居中、TF 轴、Grid 都在；Camera 面板出实时彩色图；起 `detect_rknn` 后 Detect 面板出检测图，`/debug_image` **29.9 Hz** |

### 9.1.1 板上怎么"眼见为实"（不需要人看 VNC）

板子上没有 `xwd` / `import` / `ffmpeg`，但 **libX11 在**（rviz 自己就靠它），
所以直接 ctypes 走 X11 协议抓屏即可（只用 numpy + cv2）：

```python
x11 = ctypes.CDLL("/data/local/release/lib/libX11.so.6")
dpy = x11.XOpenDisplay(b":0")
img = x11.XGetImage(dpy, root, 0, 0, w, h, AllPlanes, ZPixmap)   # XImage 结构体照抄 Xlib.h
```

拿回来 base64 落本地 PNG，就能直接看。**前提是 X 得先起来**，而这有个坑：

> 板子根文件系统是**只读**的，`/tmp` 也在上面。Xvfb 要在 `/tmp` 建锁文件，
> 所以**直接起会报** `Fatal server error: Could not create lock file in /tmp/.tX0-lock`
> 然后客户端连不上（rviz 报 `qt.qpa.xcb: could not connect to display :0`）。
> 一键菜单的 `check_mount()` 干的就是这件事，自己写脚本时**必须先照抄**：
>
> ```sh
> mount -o remount,rw /
> mount -t tmpfs tmpfs /tmp          # 根目录 remount rw 之后通常就不需要这步了
> rm -f /tmp/.tX*-lock; rm -rf /tmp/.X*-unix    # 清上次异常退出的残留
> ```
>
> 少了这步的现象是"Xvfb 进程在、socket 没有、所有客户端连不上"，很容易误判成 rviz 的问题。

> 探测过程只起了「检测节点」这一条链，**没有启动底盘与机械臂**，按 PID 清理干净。

### 9.2 还没做（要上场地，且会动物理设备）

1. 走一键菜单第 11 项跑完整 `semi_auto.launch`（会启动底盘驱动与机械臂；
   `grasp.py` 初始化时会发一次机械臂复位指令）。
2. 相机出图后看 `rostopic hz /debug_image` 的**端到端**帧率（上面的 19 ms 是
   纯推理+后处理，不含取流与解码）。
3. 拿实物确认 `/location` 发的是 `obj_id=56`（西兰花）而不是 50。
4. 把西兰花放在画面已知位置，比对 `/location` 的 `loc_centre_x/y` 与实际像素，
   定 `roi_offset` 用默认 `[160,160]` 还是复现原版的 `[160,158.4]`。
5. 另开终端 `run rosrun semi_auto_match_24 teleop.py` 逐键验证抓取/放下/复位。
6. `~show_debug:=true` 下确认 `cv2.imshow` 能不能出画面（出不来就用
   `/debug_image` + rviz 的 Detect 面板）。

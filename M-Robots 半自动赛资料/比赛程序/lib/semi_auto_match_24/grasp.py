#!/usr/bin/env python3.12
# -*- coding: utf-8 -*-

import os
import sys
import time
import random
import rospy
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from semi_auto_match_24.msg import LocationCentre

# 机械臂消息：优先用本工具链编出来的 spark_carry_object；
try:
    from spark_carry_object.msg import position, status
except ImportError:                                  # pragma: no cover
    from swiftpro.msg import position, status

# 默认的手眼标定文件（4 个数：x 的 k/b、y 的 k/b）。
HOME_CALIB_FILE = os.path.expanduser("~/thefile.txt")
BOARD_CALIB_FILE = "/data/local/release/usr/thefile.txt"

# ---------------------------------------------------------------------------
# 抓取高度（单位 **毫米** —— swiftpro 的 position.z 就是 mm，
# swiftpro_write_node.cpp 直接拼成 Gcode "G0 X.. Y.. Z.."，注释里写明 x,y,z(mm)）
#
# 水源和物块用同一个高度**，跟按的是大写还是小写无关。
DEFAULT_HEIGHT = {"z_up": 20.0, "z_down": -45.0}

# 颜色阈值（OpenCV HSV：H 0~179 / S 0~255 / V 0~255），可由 config/grasp_colors.yaml 覆盖。
DEFAULT_COLORS = {
    "water": {"lower": [95, 90, 80], "upper": [130, 255, 255]},
}


def target_of(layer):
    """把 grasp_cp 收到的 msg.data 映射成目标名，只用于打日志；认不出返回 None。

    '0'/'10' 水源、'4'/'11' 物块（西兰花）。
    '6'/'12' 原本是红色方块，本版已删除，故意不再映射（日志里会显示 None）。
    """
    if layer in ("0", "10"):
        return "water"
    if layer in ("4", "11"):
        return "broccoli"
    return None


# ---------------------------------------------------------------------------
# 可达性检查：见 nodes/arm_ik.py。
#
# 为什么需要它：标定把像素映射成机械臂坐标，但**机械臂够不到所有位置** ——
# 底座附近有一圈死区（z=20 时半径约 50~95mm 那一带），图像最上方那 1/4
# （pos.x > ~370mm）也整体超出臂展。发过去固件不接受：表现为**机械臂不动**，
# 而 grasp() 后面照样开泵、照样抬到 (230,0,160) —— 看起来就像"没抓就跑了"。
# 同时 swiftpro_rviz_node 会刷 "Inverse kinematic is wrong"（那个节点只是把位姿
# 画到 rviz，不参与实际运动，所以那句报错本身不是原因，是同一个数学结论）。
#
# 实测对齐：报错那次的 (67.18, 22.28, 20) 判不可达；日志里真正抓成功的
# (126.42, -8.58) 与 (100.78, -2.77) 判可达。
try:
    from arm_ik import ik_reachable, pixel_to_arm
except ImportError:                                    # 从源码树直接跑时
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from arm_ik import ik_reachable, pixel_to_arm


def resolve_calib_file():
    """决定用哪个标定文件：``~calib_file`` > 环境变量 > HOME > 本板落点。"""
    candidates = []
    explicit = rospy.get_param("~calib_file", "")
    if explicit:
        candidates.append(explicit)
    env = os.environ.get("SEMI_AUTO_MATCH_CALIB")
    if env:
        candidates.append(env)
    candidates.append(HOME_CALIB_FILE)
    candidates.append(BOARD_CALIB_FILE)
    for path in candidates:
        if path and os.path.exists(path):
            return path
    raise RuntimeError(
        "找不到手眼标定文件；找过: %s。用 ~calib_file:=<路径> 指定，"
        "或设 SEMI_AUTO_MATCH_CALIB，或先按原流程做一次标定生成它" % (candidates,))


class GraspObject():

    def __init__(self):
        '''
        初始化参数
        x & y 代表水源的中心点位置信息
        x_prev & y_prev 代表前一轮记录的中心点
        found_count 代表识别中心点的次数
        self.found_*** 代表是否准确找到水源的中心点
        '''

        # 定义水源参数
        self.water_found_count = 0 ; self.found_water = False 
        self.water_x = 0 ; self.water_y = 0 ; self.water_x_prev = 0 ; self.water_y_prev = 0
        
        # 定义西兰花参数
        self.broccoli_found_count = 0 ; self.found_broccoli = False
        self.broccoli_x = 0 ; self.broccoli_y = 0 ; self.broccoli_x_prev = 0 ; self.broccoli_y_prev = 0 

        # 调试窗：板端要 run_gui 才有 GL 上下文，默认关。
        self.show_debug = bool(rospy.get_param("~show_debug", False))
        self._debug_failed = False

        # 获取标定文件数据
        filename = resolve_calib_file()
        rospy.logwarn('手眼标定文件: ' + filename)
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        rospy.logwarn('X axia k and b value: ' + str(self.x_kb))
        rospy.logwarn('X axia k and b value: ' + str(self.y_kb))

        # 抓取高度：两个目标共用一套，可由 config/grasp_heights.yaml 覆盖
        self.z_up = float(rospy.get_param("~z_up", DEFAULT_HEIGHT["z_up"]))
        self.z_down = float(rospy.get_param("~z_down", DEFAULT_HEIGHT["z_down"]))
        rospy.logwarn("抓取高度(mm): z_up=%.0f z_down=%.0f（水源/物块共用）"
                      % (self.z_up, self.z_down))

        # 颜色阈值：可由 config/grasp_colors.yaml 覆盖
        self.color = {}
        for _name, _dflt in DEFAULT_COLORS.items():
            self.color[_name] = (
                np.array(rospy.get_param("~%s/lower" % _name, _dflt["lower"])),
                np.array(rospy.get_param("~%s/upper" % _name, _dflt["upper"])),
            )
        rospy.logwarn("颜色阈值(HSV): " + "；".join(
            "%s H[%d,%d] S[%d,%d] V[%d,%d]"
            % ((_n,) + tuple(self.color[_n][0]) + tuple(self.color[_n][1]))
            for _n in sorted(self.color)))

        # 发布机械臂位姿
        self.pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber('/grasp', String, self.grasp_cp, queue_size=1)
        # 机械臂 attach/detach（/swiftpro_status_topic；上电初始化用）
        self.arm_status_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        # 订阅摄像头话题,接收图像信息后跳转image_cb进行处理
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)
        # 订阅物体识别位置
        self.detector_object = rospy.Subscriber("location", LocationCentre, self.object_cb,queue_size=1)

        # 上电初始化：给机械臂发一次 attach（0→M2019 断开 → 1→M17 接上）
        self.arm_attach()
        pos = position()
        pos.x = 20
        pos.y = 150
        pos.z = 35
        self.pub1.publish(pos)


    def object_cb(self, loc):   
        print("loc_id:",loc.obj_id)  

        # 判断是否识别到西兰花 56 
        if loc.obj_id == 56:  
            rospy.loginfo("locating the broccoli")

            self.broccoli_x = loc.loc_centre_x
            self.broccoli_y = loc.loc_centre_y
            
            print("broccoli: ", self.broccoli_x, self.broccoli_y) 
            
            if self.broccoli_found_count >= 10:
                # 将找到物体的参数设定为True
                self.found_broccoli = True
            else:
                # 判断识别的水源中心点位置有无移动
                if abs(self.broccoli_x - self.broccoli_x_prev) <= 10 and abs(self.broccoli_y - self.broccoli_y_prev) <= 10:
                    self.broccoli_found_count +=  1
                else:
                    # 一旦移动，归零
                    self.broccoli_found_count = 0
                    self.found_broccoli = False
            
            self.broccoli_x_prev = self.broccoli_x
            self.broccoli_y_prev = self.broccoli_y

        

    # 对传入的一帧图像进行处理，使用CV检测物体       
    def image_cb(self, data):
        # 将 ROS image消息类型转换成opencv类型  
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
            # print(cv_image1.shape)
        except CvBridgeError as e:
            print('error')
        # 由RGB颜色转换成HSV颜色空间
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)
        # 蓝色物体颜色检测范围（可由 config/grasp_colors.yaml 覆盖）
        LowerBlue, UpperBlue = self.color["water"]
        # 阈值处理
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        # 位运算，对图像进行掩膜处理
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)
        # 取三维矩阵中第一维的所有数据
        cv_image4 = cv_image3[:, :, 0]
        # 均值滤波处理
        blurred = cv2.blur(cv_image4, (9, 9))
        # 简单阈值函数
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        # 获取结构元素
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        # 执行高级形态变换
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # 腐蚀处理
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        # 扩张处理
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4) 
        # 检测轮廓
        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 轮廓为空时 drawContours(..., 0, ...) 会断言失败，先判断
        if len(contours) > 0:
            cv2.drawContours(cv_image1, contours, 0, (0, 255, 0), 2)
        # 显示图像
        

        # 根据检测的轮廓信息，找到其中距离最近的水源
        # len(contours) 识别到的水源的个数
        if len(contours) > 0:
            dis = []    
            dis_min = 640          
            # enumerate()为迭代的对象添加序列号
            for i, c in enumerate(contours):
                # 绘制水源图形
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                # 计算水源中心点
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                # w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                # h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2)
                distance =  math.sqrt((x_mid-320)**2+(480-y_mid)**2)
                # 找出距离最近的水源
                dis.append(distance)
                if dis[i] < dis_min:    
                    dis_min = dis[i]
                    self.water_x = x_mid
                    self.water_y = y_mid
            
            # 基于有可能出现在移动或旋转过程中识别到水源的情况
            # 设定found_count参数判断水源有无移动
            # 当found_count次数达到30时，认为水源没有移动，可以进行定位           
            if self.water_found_count >= 30:
                # 将找到物体的参数设定为True
                self.found_water = True                
            else:
                # 判断识别的水源中心点位置有无移动
                if abs(self.water_x - self.water_x_prev) <= 4 and abs(self.water_y - self.water_y_prev) <= 4:
                    self.water_found_count = self.water_found_count + 1
                else:
                    # 一旦移动，归零
                    self.water_found_count = 0
        else:
            # 没有找到轮廓，判断水源移动参数归零
            self.water_found_count = 0

        self.water_x_prev = self.water_x
        self.water_y_prev = self.water_y


        # print("water_pos: ", self.water_x, self.water_y)
        
        cv2.circle(cv_image1, (int(self.water_x), int(self.water_y)), 5, (0, 0, 255), -1)

        self._show("contours", cv_image1)

    def _show(self, window, image):
        """调试窗。板端要有 GL 上下文（run_gui）才画得出来；
        画不出来只警告一次，不打断抓取流程。"""
        if not self.show_debug:
            return
        try:
            cv2.imshow(window, image)
            cv2.waitKey(1)
        except Exception as exc:
            if not self._debug_failed:
                self._debug_failed = True
                rospy.logwarn(
                    "cv2.imshow 失败（%s）；板端需要 run_gui 前缀，"
                    "或把 ~show_debug 设成 false", exc)


    def grasp_cp(self, msg):
        # 设置睡眠时间
        rate = rospy.Rate(1)
        # 接受到进行抓取水源的信号
        if msg.data == '0' or msg.data == '10':            
            # 若未能获取水源准确中心点进入此循环
            while not self.found_water:
                rospy.logwarn('finding water' )
                rate.sleep()
            # 抓取检测到的物体    
            print("water grasp on!!!!!!")
            self.grasp(self.water_x,self.water_y, msg.data)
            self.found_water = False

        # 接受到进行抓取西兰花的信号
        elif msg.data == '4' or msg.data == '11':            
            # 若未能获取西兰花准确中心点进入此循环
            while not self.found_broccoli:
                rospy.logwarn('finding broccoli')
                rate.sleep()
            # 抓取检测到的物体
            print("grasping the broccoli")
            self.grasp(self.broccoli_x,self.broccoli_y, msg.data)
            self.broccoli_found_count = 0

        # 接受到进行放置的信号
        else:
            height = int(msg.data)
            
            self.release_object(height)
            rate.sleep()

    # 执行抓取
    def grasp(self,y,x, layer):        
        print("start to grasp\n")
        # 设定睡眠时间
        r1 = rospy.Rate(0.5)
        r2 = rospy.Rate(10)

        # 高度两个目标共用一套；大小写只决定发哪条指令，不影响高度。
        z_up = self.z_up
        z_down = self.z_down
        rospy.loginfo("抓取目标=%s layer=%s -> z_up=%.0f z_down=%.0f (mm)",
                      target_of(layer), layer, z_up, z_down)


        # 物体所在坐标+标定误差
        # 控制机械臂到目标上方
        pos = position()
        # 注意本函数的形参顺序是 (y, x)，调用处传的是 (像素列, 像素行)：
        # 所以这里 x 是**行**、y 是**列**（原程序就是这么反的，没动）。
        pos.x, pos.y = pixel_to_arm(x, y, self.x_kb, self.y_kb)
        pos.z = z_up
        print("pos.x=",pos.x,"pos.y=",pos.y,"pos.z=",pos.z)

        # 够不到就别发了 —— 固件不接受，机械臂不会动，但后面照样开泵、照样抬走，
        # 现场看起来就是"没抓就跑了"。这里直接放弃，并说清该把物体挪到哪。
        if not (ik_reachable(pos.x, pos.y, z_up) and ik_reachable(pos.x, pos.y, z_down)):
            rospy.logerr(
                "抓取位姿机械臂够不到，放弃本次抓取（不吸、不抬）: "
                "像素(行=%.0f, 列=%.0f) -> 机械臂(%.1f, %.1f) z=%.0f/%.0f；"
                "把物体挪到画面可达区域再试（大约行 320~400，即画面中下部，"
                "避开最上方 1/4 和底部中央的缺口）",
                x, y, pos.x, pos.y, z_up, z_down)
            return

        self.pub1.publish(pos)
        rospy.sleep(2)

        # 控制机械臂向下运动 
        pos.x, pos.y = pixel_to_arm(x, y, self.x_kb, self.y_kb)
        pos.z = z_down
        print(pos.z)
        self.pub1.publish(pos)
        # r2.sleep()
        rospy.sleep(4)

        # 开始吸取物体
        self.pub2.publish(1)
        rospy.sleep(1)

        # 提起物体
        pos.x = 230  
        pos.y = 0
        pos.z = 160  
        self.pub1.publish(pos)
        rospy.sleep(1)

    # 放物体
    def release_object(self,height):
        r1 = rospy.Rate(1)     # 1s
        pos = position()

        if height == 1:
            pos.x = 220
            pos.y = 0
            pos.z = -40  #-80
            self.pub1.publish(pos)
            r1.sleep()

        # stop pump
        self.pub2.publish(0)
        r1.sleep()

        self.arm_to_home()


    def arm_to_home(self):
        pos = position()
        pos.x = 20;pos.y = 150;pos.z=60     # 140 0 0
        self.pub1.publish(pos)
        rospy.sleep(1)




    def arm_attach(self):
        """上电初始化：给 /swiftpro_status_topic 发 0→1。
        ``__init__`` 里跑一次，给机械臂做上电 attach。
        """
        print("arm attach\n")
        r1 = rospy.Rate(1)
        self.arm_status_pub.publish(0)
        r1.sleep()
        self.arm_status_pub.publish(1)
        r1.sleep()


        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

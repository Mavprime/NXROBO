# runtask.py -- reference excerpt from
# Manuals/CN spark configure.pdf, sections 3.3-3.6
# ("runtask.py 385", 291, 410, 450, 141).
#
# The manual shows only these methods/blocks: imports, the class definition
# and the classification-board code are not included, so this is NOT a
# complete runnable node. The working implementation is auto/codes/runtask.py.

# 接收到启动自动赛信号，开始执行任务
def run_task(self):
    ret = False  # 是否导航成功标志
    item_type = 0
    self.arm.arm_home()  # 移动机械臂到其他地方

    # ===== 现在开始执行任务 =====
    rospy.loginfo("start task now.")

    # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
    self.robot.step_go(0.2)
    if self.stop_flag:
        return

    # ==== 移动机械臂 ====
    self.arm.arm_position_reset()  # 重置机械臂坐标系
    self.arm.arm_grasp_ready()     # 移动机械臂到其他地方

    if self.robot.goto_local("Classification_area"):
        rospy.sleep(2)
    else:
        rospy.logerr("Navigation to Classification_area failed,please run task again ")
        self.stop_flag = True

    # ===== 3.4 前往分拣区，识别并抓取物体 =====
    # 创建任务安排字典，设定前往的抓取地点与次数
    sorting_status_times = {
        "Sorting_N": 1,
        "Sorting_W": 1,
    }
    sorting_name = "Sorting_N"

    # =======开始循环运行前往中心区域抓取与放置任务======
    while True:
        # 根据任务安排逐步执行
        print("readying to sort_areas")
        if sorting_status_times[sorting_name] == 0:
            sorting_status_times.pop(sorting_name)
            if len(sorting_status_times) == 0:
                break
            else:
                sorting_name = list(sorting_status_times.keys())[0]

        # ===== 导航到目标地点====
        ret = self.robot.goto_local(sorting_name)  # 导航到目标点
        rospy.sleep(2)  # 停稳

        if self.stop_flag:
            return

        # ===== 识别并抓取物体====
        item_type = 0
        if ret:  # 判断是否成功到达目标点
            print("========扫描中，准备抓取===== ")
            item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag:
                return

        if item_type == 0:  # 如果没有识别到物体，将该地点的抓取次数归0
            sorting_status_times[sorting_name] = 0
            continue

        # ===== 3.5 前往收取区，放置并堆叠物体 =====
        self.arm.arm_grasp_ready()
        print("========前往放置区===== ")
        ret = self.robot.goto_local("Collection_B")  # 根据抓到的物品类型，导航到对应的放置区
        rospy.sleep(2)  # 停稳
        if self.stop_flag:
            return

        if ret:
            self.arm.drop()          # 放下物品
            self.robot.step_back()   # 后退
            if self.stop_flag:
                return
        else:
            rospy.logerr("task error: navigation to the drop_place fails")
            self.arm.drop()
        if self.stop_flag:
            return


# 目标点导航
def goto_local(self, name):
    '''
    根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
    @return: True 为成功到达, False 为失败
    '''

    # 发布目标位置
    self.goto_local_pub.publish("go " + name)

    # 设定1分钟的时间限制，进行阻塞等待
    try:
        ret_status = rospy.wait_for_message(
            'move_base/result', MoveBaseActionResult,
            rospy.Duration(60)).status.status
    except Exception:
        rospy.logwarn("nav timeout!!!")
        ret_status = GoalStatus.ABORTED

    # 如果一分钟之内没有到达，放弃目标
    if ret_status != GoalStatus.SUCCEEDED:
        rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
            GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        try:
            rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(3))
        except Exception:
            rospy.logwarn("move_base result timeout. this is abnormal.")
        rospy.loginfo("==========Timed out achieving goal==========")
        return False
    else:
        rospy.loginfo("==========Goal succeeded==========")
        return True


# 物体抓取部分
def grasp(self):
    '''
    使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
    @return: 抓取到物品的id, 0为未识别到需要的物品
    '''
    r1 = rospy.Rate(0.25)
    r2 = rospy.Rate(10)
    x = 0
    y = 0
    z = 0

    # 寻找物品
    cube_list = self.cam.detector()
    # rospy.sleep(2)
    if len(cube_list) == 0:
        rospy.logwarn("没有找到物品啊。。。去下一个地方")
        self.grasp_status_pub.publish(String("1"))
        return 0

    # 获取机械臂目标位置
    x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
    y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
    z = -55

    print(f"找到物品了！它在: {x}, {y}, {z}")

    # 机械臂移动到目标位置上方
    self.interface.set_pose(x, y, z + 10)
    rospy.sleep(2)

    # 机械臂移动到目标位置
    self.interface.set_pose(x, y, z)
    rospy.sleep(2)

    # 打开气泵，进行吸取
    self.interface.set_pump(True)
    rospy.sleep(1)
    # r2.sleep()

    # 抬起目标方块
    print(f"我把物品抬起来了")
    self.interface.set_pose(x, y, z + 80)
    rospy.sleep(2)
    # r1.sleep()

    self.grasp_status_pub.publish(String("0"))

    return cube_list[0][0]

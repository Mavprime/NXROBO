#!/usr/bin/env python3.12
# -*- coding: utf-8 -*-

import os
import sys
import tty
import termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 全局变量（在 keyboardLoop 里 init_node 之后才真正建 Publisher）
cmd = Twist()
pub = None
grasp_pub = None


def keyboardLoop():
    global pub, grasp_pub

    rospy.init_node('teleop')

    if not sys.stdin.isatty():
        rospy.logfatal(
            "teleop 需要真终端（要 termios 读单键）。板端没有终端模拟器，"
            "被 roslaunch 拉起时 stdin 是 EOF —— 请在 MobaXterm 里另开窗口跑："
            "run rosrun semi_auto_match_24 teleop.py")
        return 1

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 10))
 
    #速度变量
    # 慢速
    walk_vel_ = rospy.get_param('walk_vel', 0.1)
    # 快速
    run_vel_ = rospy.get_param('run_vel', 0.5)
    yaw_rate_ = rospy.get_param('yaw_rate', 0.5)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.0)
    # walk_vel_前后速度
    max_tv = walk_vel_
    # yaw_rate_旋转速度
    max_rv = yaw_rate_
    # 参数初始化
    speed=0

    print ("使用[WASD]控制机器人")
    print ("按[T]抓取物体")
    print ("按[G]抓取水源")
    print ("按[H]放下物体")
    print ("按[Q]退出" )
 
    #读取按键循环
    while not rospy.is_shutdown():
        # linux下读取键盘按键
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
        #不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # ch代表获取的键盘按键
        if ch == 'g':
            msg=String()
            msg.data='0'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0
        elif ch == 't':
            msg=String()
            msg.data='4'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0
        elif ch == 'G':
            msg=String()
            msg.data='10'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0
        elif ch == 'T':
            msg=String()
            msg.data='11'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0

        elif ch == 'h':
            msg=String()
            msg.data='1'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0
        elif ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        elif ch == 'q':
            sys.exit(0)
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()
    return 0


def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    if pub is not None:
        pub.publish(cmd)


if __name__ == '__main__':
    try:
        sys.exit(keyboardLoop())
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
import rospy
from swiftpro.msg import position, status
import time

rospy.init_node("test_arm_motion", anonymous=True)
pub = rospy.Publisher("position_write_topic", position, queue_size=1)
stat_pub = rospy.Publisher("swiftpro_status_topic", status, queue_size=1)

print("Waiting for subscriber...")
timeout = time.time() + 5.0
while pub.get_num_connections() == 0 and time.time() < timeout:
    time.sleep(0.1)

print("Connections:", pub.get_num_connections())
stat_pub.publish(status(1))
time.sleep(0.5)

p = position()
p.x = 200.0
p.y = 0.0
p.z = 100.0
pub.publish(p)
print("Sent pose (200, 0, 100)")
time.sleep(1.0)

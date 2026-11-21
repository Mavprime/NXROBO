#!/usr/bin/env python3
import rospy
from swiftpro.msg import position, status
import time

rospy.init_node("verify_live_arm", anonymous=True)
pub = rospy.Publisher("position_write_topic", position, queue_size=1)
stat_pub = rospy.Publisher("swiftpro_status_topic", status, queue_size=1)

time.sleep(0.5)
stat_pub.publish(status(1))
time.sleep(0.3)

# Move arm forward
p = position()
p.x = 180.0
p.y = 0.0
p.z = 100.0
pub.publish(p)
print("Commanded arm to (180, 0, 100)")
time.sleep(1.5)

# Move arm to park
p.x = 50.0
p.y = 180.0
p.z = 160.0
pub.publish(p)
print("Commanded arm to park (50, 180, 160)")
time.sleep(1.0)
print("Done!")

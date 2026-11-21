import rospy
import time
from swiftpro.msg import position, SwiftproState

rospy.init_node("test_firmware_high_reach")
pub = rospy.Publisher("position_write_topic", position, queue_size=1)
cur_pos = [0, 0, 0]

def cb(msg):
    global cur_pos
    cur_pos = [msg.x, msg.y, msg.z]

sub = rospy.Subscriber("SwiftproState_topic", SwiftproState, cb)
time.sleep(1.0)

for z in [90.0, 80.0, 70.0, 40.0, 32.0]:
    cmd = position()
    cmd.x, cmd.y, cmd.z = 345.0, 0.0, z
    pub.publish(cmd)
    time.sleep(1.2)
    print(f"Target (345.0, 0.0, {z:4.1f}) -> Actual: ({cur_pos[0]:5.1f}, {cur_pos[1]:5.1f}, {cur_pos[2]:5.1f})")

# park safely
cmd.x, cmd.y, cmd.z = 50, 180, 160
pub.publish(cmd)
time.sleep(0.8)
print("Parked.")

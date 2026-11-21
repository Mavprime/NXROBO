import rospy
import time
from swiftpro.msg import position, SwiftproState

rospy.init_node("test_firmware_max_reach")
pub = rospy.Publisher("position_write_topic", position, queue_size=1)
cur_pos = [0, 0, 0]

def cb(msg):
    global cur_pos
    cur_pos = [msg.x, msg.y, msg.z]

sub = rospy.Subscriber("SwiftproState_topic", SwiftproState, cb)
time.sleep(1.0)

for tx in [320, 335, 345, 350, 355, 360]:
    cmd = position()
    cmd.x, cmd.y, cmd.z = tx, 0, 40
    pub.publish(cmd)
    time.sleep(1.0)
    print(f"Target X={tx:5.1f} -> Actual: ({cur_pos[0]:5.1f}, {cur_pos[1]:5.1f}, {cur_pos[2]:5.1f})")

# park safely
cmd.x, cmd.y, cmd.z = 50, 180, 160
pub.publish(cmd)
time.sleep(0.8)
print("Parked.")

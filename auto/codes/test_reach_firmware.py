import rospy
import time
from swiftpro.msg import position, SwiftproState

rospy.init_node("test_firmware_reach")
pub = rospy.Publisher("position_write_topic", position, queue_size=1)
cur_pos = [0, 0, 0]

def cb(msg):
    global cur_pos
    cur_pos = [msg.x, msg.y, msg.z]

sub = rospy.Subscriber("SwiftproState_topic", SwiftproState, cb)
time.sleep(1.0)

test_targets = [
    (180, 0, 80),
    (240, 0, 50),
    (270, 0, 40),
    (290, 0, 40),
    (310, 0, 40),
    (320, 0, 40),
    (330, 0, 40),
    (200, 0, 100),
]

for tx, ty, tz in test_targets:
    cmd = position()
    cmd.x, cmd.y, cmd.z = tx, ty, tz
    pub.publish(cmd)
    time.sleep(1.2)
    print(f"Target: ({tx:5.1f}, {ty:5.1f}, {tz:5.1f}) -> Actual: ({cur_pos[0]:5.1f}, {cur_pos[1]:5.1f}, {cur_pos[2]:5.1f})")

# park safely
cmd.x, cmd.y, cmd.z = 50, 180, 160
pub.publish(cmd)
time.sleep(1.0)
print("Parked.")

import sys
sys.path.append('/home/spark/codes')
from runtask import CamAction
from match_logic import pixel_to_xy_at_z
cam = CamAction()
print("Top face (213, 230) at z=-50 :", pixel_to_xy_at_z(cam.transform3d, 213, 230, -50))
print("Top face (213, 230) at z=-100:", pixel_to_xy_at_z(cam.transform3d, 213, 230, -100))

from math import pi
from geometry_msgs.msg import Pose, Point

#unit in mm
L1 = 100
L2 = 117.5
L3 = 95
L4 = 91.46

#pitch angle is fixed in -60 degree
pitch_angle = -pi/3

# Camera location relative robot base and info
serial = '31702051'
camera_x = 190
camera_y = 0
camera_z = 400
cam_h = 640
cam_w = 512

# Out of range values for collission checking and other vision params
out_of_reach_x = 200
block_offset = 16

# Object class
class Zone:
    def __init__(self, colour, point: Point) :
        self.colour = colour
        self.pose = Pose(
            position = point
        )

zone_1 = Point(-50, 150, 0)
zone_2 = Point(-150, 50, 0)
zone_3 = Point(-150, 50, 0)
zone_4 = Point(-150, -50, 0)

red_zone = Zone("red", zone_1)
blue_zone = Zone("blue", zone_2)
green_zone = Zone("green", zone_3)
yellow_zone = Zone("yellow", zone_4)


# State positions
#Setup
setup_point = Point(0, 0, L1+L2+L3+L4)
setup_pose = Pose(
    position=setup_point
)

# Positions for the starting stage, need to be close enough for standard grabbing range
init_point = Point(100, 0, 100)
init_pose = Pose(
    position=init_point
)

# Positions for the identification stage, needs to be high vertically and centred about camera axis
id_point = Point(190, 0, 150)
id_pose = Pose(
    position=setup_point
)




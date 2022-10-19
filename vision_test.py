from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Transform

from ast import increment_lineno
from constants import *

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

import modern_robotics as mr

T = Transform()
T.translation.x = -0.02e03
T.translation.y = 0.043e03
T.translation.z = 0.429e03

T.rotation.x = -0.993e3
T.rotation.y = -0.078e3
T.rotation.z = -0.087e3

T_rc = np.array([
            [0, 1,  0, camera_x],
            [1, 0,  0, camera_y],
            [0, 0, -1, camera_z],
            [0, 0,  0, 1]
        ])

def camera_to_base(fid_t):
    p1 = np.array([fid_t.translation.x, fid_t.translation.y, fid_t.translation.z])
    # R1 = np.array([[fid_t.rotation.x,0,0],[0,fid_t.rotation.y,0],[0,0,fid_t.rotation.z]])
    R1 = np.array([[1,0,0],[0,1,0],[0,0,1]])

    t_1 = mr.RpToTrans(R1, p1)
    t_2 = T_rc @ t_1

    R2, p2 = mr.TransToRp(t_2)

    return p2

print(camera_to_base(T))
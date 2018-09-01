import rospy
import math
import numpy as np

from COLREGS_utils import functions

HeadingRangeConvert360 = functions.HeadingRangeConvert360

###------- quaternion_to_euler_angle -------###

def quaternion_to_euler_angle(x, y, z, w):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

def angle_to_yaw(teta):

    teta = HeadingRangeConvert360(teta) #makes the angle to be in the limits of [0,360)
    yaw = 0 #initialize. Some errors occured with the 3rd if condition saying "yaw is still not defined"

    if (teta < 90 and teta >= 0):
		yaw = (math.pi / 180) * (teta - 90)
    elif (teta >= 90 and teta < 180):
	    yaw = (math.pi / 2) - (math.pi / 180) * (180 - teta)
    elif (teta >= 180 and teta < 270):
	    yaw = math.pi - (math.pi / 180) * (270 - teta)
    elif (teta >= 270 and teta < 360):
	    yaw = - (math.pi / 2) - (math.pi / 180) * (360 - teta)

    if (yaw > math.pi): yaw = 2 * math.pi - yaw
    elif (yaw < -math.pi): yaw = - 2 * math.pi - yaw

    return yaw

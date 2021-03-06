#!/usr/bin/env python

###---------Import modules----------###

import rospy #need to import rospy if you are writing a ROS Node
import math
import numpy as np
from ceiia_asv_msgs.msg  import AIS,debug,desired,Course
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from COLREGS_utils import functions, quat_euler_yaw, main_algorithm, lat_lon_convert

HeadingRangeConvert360 = functions.HeadingRangeConvert360
quaternion_to_euler_angle = quat_euler_yaw.quaternion_to_euler_angle #Convert quaternion to euler and get heading
geo_utm = lat_lon_convert.geo_utm #Function to convert lat and lon coordinates to local coordinates
Algorithm_1 = main_algorithm.Algorithm_1


class Vehicle_status(object):
    """docstring for Vehicle_status."""
    def __init__(self):
        super(Vehicle_status, self).__init__()
        ##Initialize object: Ownship or Contact##
        self.mmsi = None
        self.lat = None
        self.lon = None
        self.x = None  # [m]
        self.y = None  # [m]
        self.COG = None  # [degrees]
        self.SOG = None # [m/s]
        self.lenght = None  # [m]

        #Initialize mode and submode
        self.mode = 0
        self.submode = 0

    def ownship(self):
        self.mmsi = 123456789
        self.lat = 41.18
        self.lon = -8.70
        self.x = 0  # [m]
        self.y = 0  # [m]
        self.COG = 180  # [degrees]
        self.SOG = 0.5 # [m/s]
        self.lenght = 3  # [m]

    def contact(self):
        self.mmsi = 999999999
        self.lat = 41.18080
        self.lon = -8.703605
        self.x = -45.0  # [m]
        self.y = -105.0  # [m]
        self.COG = 20  # [degrees]
        self.SOG = 4 # [m/s]
        self.lenght = 7.5  # [m]

    def callback(self, msg): #Contact AIS data
        self.mmsi = msg.mmsi

        if self.mmsi == 123456789:
            self.lat = msg.Latitude
            self.lon = msg.Longitude
            self.SOG = msg.SOG
            self.COG = msg.COG
            self.lenght = msg.lenght
            self.mode = msg.mode

    def update_xy(self, x_cn, y_cn): #Contact AIS data

        self.x = x_cn
        self.y = y_cn

    def callback_2(self, msg): #Update Ownship pose and twist values

        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y

        self.SOG = math.sqrt(v_x ** 2 + v_y ** 2)

        x_e,y_e,z_e = quaternion_to_euler_angle(msg.pose.pose.orientation.x, \
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        teta = z_e * (-1) + 90
        self.COG = HeadingRangeConvert360(teta)

    def callback_3(self, msg): #Update Ownship pose and twist values

        self.lat = msg.latitude
        self.lon = msg.longitude

class Parameters(object):
    """docstring for Vehicle_status."""
    def __init__(self):
        super(Parameters, self).__init__()
        ##Reference Parameters:
        self.lat_0 = 41.1812101
        self.lon_0 = -8.7048601
        self.fi_headon = 20 #12 # [degrees]
        self.g_max = 100
        self.g_min = 0
        self.Safety_interval = 2
        self.Start_Manouver = 6.5

def talker(heading,velocity,mmsi): #ROS Publisher #Sets the desired velocity and yaw to the control module

    if mmsi == 123456789:
        vid_number = "1"
    else:
        vid_number = "2"

    pub = rospy.Publisher('/ceiia/internal/ctl_fbw', Course, queue_size=10)
    rate = rospy.Rate(3) # 1hz
    desired.velocity = velocity
    desired.yaw = heading #FIXME: The algorithm has the yaw value inversed with gazebo, right now the *(-1) is fixing the problem
    desired.vid = vid_number
    pub.publish(desired,False) #Course message accepts 2 inputs, one called desired which has .velocity and .yaw values.
    rate.sleep()

def Ownship_subscriber(): #ROS Subscriber #Subscribes the AIS data from other vehicles
    rospy.Subscriber("chatter", AIS, Ownship.callback)
    r = rospy.Rate(1) # 1hz

def talker_AIS(mmsi,Longitude,Latitude,SOG,COG,lenght,mode): #ROS Publisher #Publishes the Ownship AIS data
    pub = rospy.Publisher('chatter', AIS, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    pub.publish(mmsi,Longitude,Latitude,SOG,COG,lenght,mode)
    rate.sleep()

def Odometry_subscriber(): #ROS Subscriber #Subscribes the odometry treated with the kalman filter
    rospy.Subscriber("p3d_odom_2", Odometry, Contact.callback_2) #topic,message,function
    r = rospy.Rate(1) # 1hz


def gps_subscriber(): #ROS Subscriber #Subscribes the odometry treated with the kalman filter
    rospy.Subscriber("gps/fix_2", NavSatFix, Contact.callback_3) #topic,message,function
    r = rospy.Rate(1) # 1hz


if __name__ == '__main__':
    rospy.init_node('Contact_AIS', anonymous=False) #Initialize node which is called COLREGS
    r = rospy.Rate(3) # 1hz

    ref = Parameters()
    Contact = Vehicle_status()
    Ownship = Vehicle_status()
    Contact.contact()
    Ownship.ownship()

    while not rospy.is_shutdown():

        node_os = Ownship_subscriber() #Subscribe AIS messages from the /chatter topic and go to the callback function
        x_os,y_os = geo_utm(ref.lat_0,ref.lon_0,Ownship.lat,Ownship.lon) #Update local coordinates of the Contact vehicle
        Ownship.update_xy(x_os,y_os) #Append to the object Contact the properties "x,y"

        r_cpa_min = Ownship.lenght * ref.Safety_interval #Inner perimeter
        r_cpa_max = Ownship.lenght * ref.Start_Manouver #Outer perimeter
        r_pwt = r_cpa_max  #FIXME: r_pwt will be equal to r_cpa_max??? Still needs to be decided

        node = Odometry_subscriber() #Subscribe odometry
        node_2 = gps_subscriber() #Subscribe lat and lon

        Contact.mode, Contact.submode,teta_d,v_d,f_function_final, yaw,mmsi = Algorithm_1(Contact,Ownship,r_cpa_max,\
                                                                                        r_cpa_min,r_pwt,ref,Contact.mmsi)
        talker(yaw,v_d,mmsi)

        talker_AIS(Contact.mmsi,Contact.lon,Contact.lat,Contact.SOG,Contact.COG,Contact.lenght,Contact.mode)

        r.sleep()

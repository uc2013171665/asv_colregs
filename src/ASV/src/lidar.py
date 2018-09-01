#!/usr/bin/env python

###---------Import modules----------###

import rospy #need to import rospy if you are writing a ROS Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ceiia_asv_msgs.msg  import desired, Course, debug2
from COLREGS_utils import quat_euler_yaw, functions

import random
import time
import sys

ranges = [None]*60
angle_increment = 0.0532473213971

quaternion_to_euler_angle = quat_euler_yaw.quaternion_to_euler_angle #Convert quaternion to euler and get heading
angle_to_yaw = quat_euler_yaw.angle_to_yaw
HeadingRangeConvert360 = functions.HeadingRangeConvert360

###-------------Debug ROS message-----------------###

def debugg(counter,danger,dist_to_goal,x,y,v,w,yaw):
    pub2 = rospy.Publisher('DEBUG2', debug2, queue_size=10)
    rate = rospy.Rate(2) # 1hz
    pub2.publish(counter,danger,v,w,yaw,x,y,dist_to_goal)
    rate.sleep()

def talker(heading,velocity): #ROS Publisher #Sets the desired velocity and yaw to the control module
    pub = rospy.Publisher('/ceiia/internal/ctl_fbw', Course, queue_size=10)
    rate = rospy.Rate(2) # 1hz
    desired.velocity = velocity
    desired.yaw = heading #FIXME: The algorithm has the yaw value inversed with gazebo, right now the *(-1) is fixing the problem
    pub.publish(desired,False) #Course message accepts 2 inputs, one called desired which has .velocity and .yaw values.
    rate.sleep()

def Ownship_subscriber(): #ROS Subscriber #Subscribes the odometry treated with the kalman filter
    rospy.Subscriber("p3d_odom", Odometry, Ownship.callback_2) #topic,message,function
    r = rospy.Rate(2) # 1hz

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 60.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_dyawrate = 60.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.5  # [m/s]
        self.yawrate_reso = 0.5 * math.pi / 180.0  # [rad/s]
        self.dt = 1.0  # [s]
        self.predict_time = 4.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 5.0  # [m]


def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    #  print(dw)

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.5
    best_traj = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            #print("ob_cost = %s for y equal to %s:" % (ob_cost,y))

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj
    #  input()
    print "best_traj:",best_traj
    print "min_u:", min_u
    dim = best_traj.shape
    if dim[0] == 1:
        min_u = [best_traj[0,3],best_traj[0,4]]
        print "min_u:", min_u
    return min_u #, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collisiton

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control
    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, goal, ob) #, traj

    return u #, traj

###------- Class to simplify dynamic and static data ------- ###

class Vehicle_status(object):
    """docstring for Vehicle_status."""
    def __init__(self):
        super(Vehicle_status, self).__init__()
        ##Initialize object: Ownship or Contact##
        self.lat = 41.18
        self.lon = -8.70
        self.x = 0  # [m]
        self.y = 0  # [m]
        self.COG = 180  # [degrees]
        self.SOG = 0.5 # [m/s]
        self.lenght = 5  # [m]
        self.GiveWay = False  # [bool]
        self.StandOn = False # [bool]
        self.w = 0 # [rad/s]

    def callback_2(self, msg): #Update Ownship pose and twist values
        # FIXME: Find a way to not use global variables

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y

        self.SOG = math.sqrt(v_x ** 2 + v_y ** 2)

        x_e,y_e,z_e = quaternion_to_euler_angle(msg.pose.pose.orientation.x, \
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        teta = z_e * (-1) + 90
        self.COG = HeadingRangeConvert360(teta)

        w_x = msg.twist.twist.angular.x
        w_y = msg.twist.twist.angular.y
        self.w = math.sqrt(w_x ** 2 + w_y ** 2)

def callback(msg): #Contact AIS data

    global ranges
    global angle_increment

    angle_increment = msg.angle_increment
    ranges = msg.ranges

def ranges_ind(ranges,number_of_points,Ownship,obstacle):
    danger = False #Initialize always as false
    for i in range(number_of_points):
        if (ranges[i] <= 30 and ranges[i] != None):
            x_o = Ownship.x + math.sin(math.radians(Ownship.COG + 90 - (3 * i))) * ranges[i]
            y_o = Ownship.y + math.cos(math.radians(Ownship.COG + 90 - (3 * i))) * ranges[i]
            danger = True
            if [int(x_o),int(y_o)] not in obstacle:
                obstacle.append([int(x_o),int(y_o)])
    return obstacle, danger

def goals(Ownship, counter):

    x_goal = Ownship.x +  math.sin(math.radians(Ownship.COG)) * 50
    y_goal = Ownship.y +  math.sin(math.radians(Ownship.COG)) * 50

    goal = ([int(x_goal), int(y_goal)])

    return goal

def lidar_data(): #ROS Subscriber #Subscribes the AIS data from other vehicles
    rospy.Subscriber("mybot/laser/scan", LaserScan, callback)
    r = rospy.Rate(2) # 1hz

if __name__ == '__main__':
    rospy.init_node('Hokuyo', anonymous=False) #Initialize node which is called Hokuyo
    r = rospy.Rate(2) # 1hz

    counter = 0
    obstacle = [[100, 100]] #FIXME: I have to give an initial obstacle to prevent error. Need fix.

    Ownship = Vehicle_status() #Initialize the Ownship Object
    goal = np.array([-50,-103]) #Goal given for debbuging
    node = lidar_data() #Subscribes the lidar data --> It was firstly called here because we need the number of points
    number_of_points = int( (180 / (angle_increment * 360 / ( 2 * math.pi ) ) ) + 1 ) #Number of points from the lidar measurments

    u = np.array([0.0, 0.0]) #Initialize the velocities for the DWA
    ob = np.matrix(obstacle) #Create a matrix for the obstacles

    config = Config()

    while not rospy.is_shutdown():

        counter +=1

        node = lidar_data() #Subscribes the lidar data
        node_2 = Ownship_subscriber() # - update ownship data

        #With the lidar_data and previous obstacles, update the obstacle variable
        obstacle, danger = ranges_ind(ranges,number_of_points,Ownship,obstacle)
        ob = np.matrix(obstacle)

        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x_inst = [Ownship.x, Ownship.y, - angle_to_yaw(Ownship.COG), Ownship.SOG, Ownship.w]
        x = np.array(x_inst)

        print """Beginning:
        ------------------------------
        """
        print("the goal is %s and counter is %s" % (goal, counter))
        print("I am in x:%s and y:%s, the yaw is %s, being the velocity %s and the rate %s" % (Ownship.x, Ownship.y, - angle_to_yaw(Ownship.COG), Ownship.SOG, Ownship.w))

        dist_to_goal = math.sqrt((Ownship.x - goal[0])**2 + (Ownship.y - goal[1])**2)

        if (dist_to_goal > config.robot_radius):

            start_time = time.time()
            #u = np.array([0.0, 0.0]) #Initialize the velocities for the DWA
            print "before u:", u
            u = dwa_control(x, u, config, goal, ob) #, ltraj
            print "after u:", u
            x = motion(x, u, config.dt)

            talker(-x[2],x[3])
            print("I have sent yaw:%s and velocity:%s and the theorethical x is %s and y is %s" % (-x[2], x[3], x[0], x[1]))
            print("However the real values --> yaw:%s and velocity:%s and the real x is %s and y is %s" % (- angle_to_yaw(Ownship.COG), Ownship.SOG, Ownship.x, Ownship.y))
            print ob
            dist_to_goal = math.sqrt((Ownship.x - goal[0])**2 + (Ownship.y - goal[1])**2)
            node_3 = debugg(counter,danger,dist_to_goal,x[0],x[1],x[3],x[4],-x[2])

            end_time = time.time()
            print("total time taken on loop: ", end_time - start_time)

            # check goal
            if dist_to_goal <= config.robot_radius:
                print """

                Goal!!

                """
                talker(-x[2],0)

        if danger == False:
            for i in range(len(ob[:, 0])):
                ox = ob[i, 0]
                oy = ob[i, 1]

                rang = math.sqrt( (Ownship.x - ox) ** 2 + (Ownship.y - oy) ** 2 )
                if rang < 30:
                    danger = True
                    break
                else:
                    danger = False
            if danger == False:
                    obstacle = [[100, 100]]
                    ob = np.matrix(obstacle)

        r.sleep()

#rospy.spin()

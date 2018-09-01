#!/usr/bin/env python

###---------Import modules----------###

import rospy #need to import rospy if you are writing a ROS Node
import math
import numpy as np
from COLREGS_utils import mode_algorithms, quat_euler_yaw, functions                       #, plot_3D

import sys

###--------Import functions from other python codes-------------###

Algorithm_2 = mode_algorithms.Algorithm_2 #Get mode and submode algorithm used in Algorithm_1 function
#plot_f_function = plot_3D.plot_f_function #For plotting the output of the f_function which is 3D ## FIXME: Not Working
quaternion_to_euler_angle = quat_euler_yaw.quaternion_to_euler_angle #Convert quaternion to euler and get heading
angle_to_yaw = quat_euler_yaw.angle_to_yaw #Simple heading convertion to yaw

##--------From COLREGS_utils import functions---------##

HeadingRangeConvert360 = functions.HeadingRangeConvert360
HeadingRangeConvert180 = functions.HeadingRangeConvert180
heading_deviation = functions.heading_deviation
delta_teta = functions.delta_teta
range_func = functions.range_func
Bearing = functions.Bearing
alpha_and_beta = functions.alpha_and_beta
vel_in_direction = functions.vel_in_direction
range_rate_function = functions.range_rate_function
tangent_heading = functions.tangent_heading
speed_in_tang_head = functions.speed_in_tang_head
Bearing_rate = functions.Bearing_rate
fore_os_cn = functions.fore_os_cn
aft_os_cn = functions.aft_os_cn
fore_cn_os = functions.fore_cn_os
aft_cn_os = functions.aft_cn_os
port_os_cn = functions.port_os_cn
star_os_cn = functions.star_os_cn
port_cn_os = functions.port_cn_os
star_cn_os = functions.star_cn_os
Crossing_relations_function = functions.Crossing_relations_function
OwnshipCrossingContact = functions.OwnshipCrossingContact
Passing_relations_function = functions.Passing_relations_function
numericalCrossing = functions.numericalCrossing
OwnshipPassingContact = functions.OwnshipPassingContact
ks = functions.ks
CPA = functions.CPA
turn_func = functions.turn_func

###--------------------- The Update Function -----------------------###
#Combines all the previous functions to generate the data in order
#to set the collision avoidance mode and submode

def Update(Ownship,Contact):
    r = range_func(Ownship,Contact)
    pre_beta = Bearing(Ownship,Contact)
    beta = alpha_and_beta(pre_beta,Ownship.COG)
    pre_alpha = Bearing(Contact,Ownship)
    alpha = alpha_and_beta(pre_alpha,Contact.COG)
    v_os_cn = vel_in_direction(beta,Ownship.SOG)
    v_cn_os = vel_in_direction(alpha,Contact.SOG)
    range_rate = range_rate_function(v_os_cn,v_cn_os)
    teta_tn = tangent_heading(pre_beta)
    teta_os_tn = heading_deviation(Ownship.COG,teta_tn)
    teta_cn_tn = heading_deviation(Contact.COG,teta_tn)
    v_os_tn = speed_in_tang_head(teta_os_tn,Ownship.SOG)
    v_cn_tn = speed_in_tang_head(teta_cn_tn,Contact.SOG)
    beta_rate = Bearing_rate(v_os_tn,v_cn_tn,r)
    fore_os_cn_ = fore_os_cn(alpha)
    aft_os_cn_ = aft_os_cn(alpha)
    fore_cn_os_ = fore_cn_os(beta)
    aft_cn_os_ = aft_cn_os(beta)
    port_os_cn_ = port_os_cn(alpha)
    star_os_cn_ = star_os_cn(alpha)
    port_cn_os_ = port_cn_os(beta)
    star_cn_os_ = star_cn_os(beta)
    teta_gamma_os, v_gamma_os, r_gamma = Crossing_relations_function(Ownship,Contact,port_os_cn_,pre_beta,r,Ownship.COG,Ownship.SOG)
    teta_gamma_cn, v_gamma_cn, r_gamma_cn = Crossing_relations_function(Contact,Ownship,port_cn_os_,pre_beta,r,Contact.COG,Contact.SOG)
    Cross_xcn, Cross_xcnb, Cross_xcns = OwnshipCrossingContact(alpha,v_gamma_os,port_os_cn_,star_os_cn_,beta_rate)
    Cross_xos, Cross_xosb, Cross_xoss = OwnshipCrossingContact(alpha,v_gamma_cn,port_cn_os_,star_cn_os_,beta_rate)
    teta_eps,v_cnh,v_eps,r_eps = Passing_relations_function(Ownship,Contact,fore_os_cn_,pre_beta,r,Ownship.COG,Ownship.SOG)
    r_xcnb, r_xcns = numericalCrossing(r_gamma,v_gamma_os,v_cnh,Contact.SOG,Cross_xcnb,Cross_xcns,r_eps)
    teta_eps_os,v_osh,v_eps_os,r_eps_os = Passing_relations_function(Contact,Ownship,fore_cn_os_,pre_beta,r,Contact.COG,Contact.SOG)
    pass_cn, pass_cnp, pass_cns = OwnshipPassingContact(v_eps,aft_os_cn_,beta_rate,fore_os_cn_)
    pass_os, pass_osp, pass_oss = OwnshipPassingContact(v_eps_os,aft_cn_os_,beta_rate,fore_cn_os_)
    k_0,k_1,k_2 = ks(Ownship,Contact,Ownship.COG,Ownship.SOG)
    t_cpa,r_tcpa = CPA(k_0,k_1,k_2,range_rate)

    return r,alpha,range_rate,beta,pass_cn,pass_cns,aft_os_cn_,star_os_cn_,star_cn_os_,port_cn_os_,Cross_xcnb,r_xcnb,Cross_xos,t_cpa,r_tcpa,pass_cnp

###-------------- CPA boolean ------------------###
def risk_assessment(r_tcpa,r_cpa_max,r_cpa_min,r):
    if r_tcpa < r_cpa_min :
        risk_cpa = 1
    else:
        risk_cpa = 0

    if (r < r_cpa_max and risk_cpa == 1):
        start_manouver_boolean = 1
    else:
        start_manouver_boolean = 0
    return risk_cpa,start_manouver_boolean

###---------- unitary Cost function ------------###

def g_avoid(r_tcpa,r_cpa_max,r_cpa_min,g_max,g_min):
    if (r_tcpa >= r_cpa_max):
        g_avoid = g_max
    elif (r_tcpa <= r_cpa_min):
        g_avoid = g_min
    else:
        g_avoid = g_min + (g_max - g_min) * ((r_tcpa - r_cpa_min) / (r_cpa_max - r_cpa_min))
    return g_avoid

###------------ Heuristic Function -------------###

def Heuristic_Overtaking(submode,pass_cnp,pass_cns):
    if (submode == 1 and pass_cnp == 1):
        h = 1
    elif (submode == 2 and pass_cns == 1):
        h = 1
    else:
        h = 0
    return h

def Heuristic_Head_On(pass_cnp):
    if (pass_cnp == 1):
        h = 1
    else:
        h = 0
    return h

def Heuristic_Crossing(submode,star_os_cn_,Cross_xcnb,port_os_cn_,turn_port,turn_star):
    if (submode == 3):
        if (turn_star == 1 and Cross_xcnb == 0):
            h = 0
        else:
            h = 1
    elif (submode == 4):
        if (turn_port == 1):
            h = 0
        elif (Cross_xcnb == 1):
            h = 0
        else:
            h = 1
    else:
        h = 0
        print """



                Major error in submodes




        """
    return h

###-------- Algorithm 1 ----------###
##Top Level onRunState() Function##

def Algorithm_1(Ownship,Contact,r_cpa_max,r_cpa_min,r_pwt,ref,mmsi):

    r,alpha,range_rate,beta,pass_cn,pass_cns,aft_os_cn_,star_os_cn_,star_cn_os_,port_cn_os_,Cross_xcnb,\
    r_xcnb,Cross_xosb,t_cpa,r_tcpa,pass_cnp = Update(Ownship,Contact)

    risk_cpa,start_manouver_boolean = risk_assessment(r_tcpa,r_cpa_max,r_cpa_min,r)

    if mmsi == 123456789:
        mode, submode = Algorithm_2(r,r_pwt,Ownship.mode,Ownship.submode,alpha,range_rate,beta,pass_cn,pass_cns,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,ref.fi_headon,port_cn_os_,Cross_xcnb,r_cpa_min,r_xcnb,Cross_xosb,start_manouver_boolean)
    else:
        print "MODE IS ",Contact.mode
        if Contact.mode == 0:
            print "Mode was just defined as zero here!!!"
            mode = 0
            submode = 0
        if Contact.mode == 3:
            mode = 3
            submode = 0
        elif Contact.mode == 2:
            mode = 4
            submode = 0
        elif Contact.mode == 5:
            mode = 6
            submode = 0
        else:
            mode, submode = Algorithm_2(r,r_pwt,Ownship.mode,Ownship.submode,alpha,range_rate,beta,pass_cn,pass_cns,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,ref.fi_headon,port_cn_os_,Cross_xcnb,r_cpa_min,r_xcnb,Cross_xosb,start_manouver_boolean)

    #If HeadOn, Crossing and Passing GiveWay situations:
    if (mode == 2 or mode == 3 or mode == 5):
        h_angles,f_function_final, teta_d, f_d, v_d = f_function(Ownship,Contact,r_cpa_max,r_cpa_min,ref.g_max,ref.g_min,r,mode,submode) #g_avoid_values
        print "f_d", f_d
    elif mode == 1: #CPA_mode
        h_angles,f_function_final, teta_d, f_d, g_avoid_values, v_d = CPA_mode(Ownship,Contact,r_cpa_max,r_cpa_min,ref.g_max,ref.g_min,r,mode,submode)

    elif (mode == 4 or mode == 6):

        teta_d = HeadingRangeConvert360(Ownship.COG)
        v_d = Ownship.SOG

        f_function_final = []
        h_angles = []
        g_avoid_values = []
        f_d = 100
        for i in range(360):
            if i == int(teta_d):
                f_function_final.append(100)
                h_angles.append(1)
                g_avoid_values.append(100)
            else:
                f_function_final.append(0)
                h_angles.append(0)
                g_avoid_values.append(0)

    if mode == 0:
        if mmsi == 123456789:

            if (sys.argv[1] == "3" or sys.argv[1] == "6"):
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 200 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 1.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []
            elif (sys.argv[1] == "5"):
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 145 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 1.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []
            else:
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 210 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 1.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []
        else:
            if (sys.argv[1] == "3" or sys.argv[1] == "6"):
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 20 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 1.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []
            elif (sys.argv[1] == "5"):
                print "Here am I!!!"
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 90 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 0.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []
            else:
                # FIXME: Add here the waypoint following algorithm and remove the values below!
                teta_d = 0 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
                v_d = 1.5 # FIXME: for debbuging, remove me
                f_d = 0 # FIXME: for debbuging, remove me
                f_function_final = []

    #if mmsi != 123456789:
    #    teta_d = 135 # FIXME: for debbuging, remove me  #200 for HeadOn and StandOnX
    #    v_d = 0.5 # FIXME: for debbuging, remove me
    #    f_d = 0 # FIXME: for debbuging, remove me
    #    f_function_final = []

    try:
        teta_d = Ownship.COG + (teta_d - Ownship.COG) / 8
        yaw = angle_to_yaw(teta_d)
        print "mmsi:",mmsi,"--> teta_d =", teta_d, "and thus the yaw =", yaw
        print "Mode at end:", mode

    except rospy.ROSInterruptException:
        pass

    return mode, submode,teta_d,v_d,f_function_final, yaw,mmsi

def Determine_angle(Ownship,Contact,r_cpa_max,r_cpa_min,g_max,g_min):
    #Gerates the g function for the 360 degrees heading with the CPA method

    teta_os = Ownship.COG
    teta_init = teta_os
    v_os = 0

    g_avoid_angles = np.matrix([[0.00 for x in range(7)] for y in range(360)])
    g_avoid_angles_correct = np.matrix([[0.00 for x in range(7)] for y in range(360)])

    for k in range(7):
        for i in range(360):
            teta_os += 1
            teta_os = HeadingRangeConvert360(teta_os)
            r = range_func(Ownship,Contact)

            pre_beta = Bearing(Ownship,Contact)
            beta = alpha_and_beta(pre_beta,teta_os)
            pre_alpha = Bearing(Contact,Ownship)
            alpha = alpha_and_beta(pre_alpha,Contact.COG)
            v_os_cn = vel_in_direction(beta,v_os)
            v_cn_os = vel_in_direction(alpha,Contact.SOG)
            range_rate = range_rate_function(v_os_cn,v_cn_os)

            k_0,k_1,k_2 = ks(Ownship,Contact,teta_os,v_os)
            t_cpa,r_tcpa = CPA(k_0,k_1,k_2,range_rate)

            g_avoid_angles[i,k] = g_avoid(r_tcpa,r_cpa_max,r_cpa_min,g_max,g_min) #Append row of headings in a specific velocity
        v_os += 0.5

    for m in range(7):
        for j in range(360):
            if j == 0:
                teta_os = 360 - teta_init
                teta_os = HeadingRangeConvert360(teta_os)
                teta_os = int(teta_os)
            else:
                teta_os += 1
                teta_os = HeadingRangeConvert360(teta_os)
                teta_os = int(teta_os)
            g_avoid_angles_correct[j,m] = g_avoid_angles[teta_os,m]

    return g_avoid_angles_correct, g_avoid_angles #r_tcpa_correct, k_0_vector_correct, k_1_vector_correct, k_2_vector_correct

def Maximize_f(teta_init,v_init,f_function_correct,mode,v_cn): #Function that loops the f_function to get the desired values
    index_v_init = min([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], key=lambda x:abs(x-v_init)) * 2
    #print "index_v_init =", index_v_init, "for v_init equal to", v_init
    for h in range(7):
        if h == 0:
            v_os = 0
        for k in range(360):
            if (k == 0 and h == 0): #FOR Start condition
                teta_os = 0 #Same as k index
                teta_d = teta_init #Initialize the heading to the original
                v_d = v_init #Initialize the velocity to the original
                f_d = f_function_correct[int(teta_init),index_v_init] #Initialize f_d value to the original heading and velocity
            elif k == 0: #Every time the for loop is completed, the teta_os variable is Initialized
                teta_os = 0
            else:
                teta_os += 1 #We start with teta_os = 1 and end in 360 = 0
            if ((mode == 2 and v_os <= v_cn) or  (mode == 2 and v_os < v_init)):
                v_os = v_os
            else:
                if f_function_correct[k,h] == f_d: #This way we ensure the closest teta and velocity to the original, when both have the same f values
                    if ((abs(teta_d-teta_init) > abs(teta_os-teta_init)) and (abs(v_d-v_init) > abs(v_os-v_init))):
                        teta_d = teta_os
                        v_d = v_os
                if f_function_correct[k,h] > f_d: #When the f_value is bigger, we change the desired heading and velocity
                    f_d = f_function_correct[k,h]
                    teta_d = teta_os
                    v_d = v_os
        v_os += 0.5 #We start with v_os = 0 and end in 3
    return f_d,teta_d,v_d

##-------generic f function----------##

def f_function(Ownship,Contact,r_cpa_max,r_cpa_min,g_max,g_min,r,mode,submode):
    #Takes the g function and combines with the respective Heuristic.
    #Returns the desired heading

    teta_os = Ownship.COG
    teta_init = teta_os
    v_os = 0
    v_init = Ownship.SOG

    v_cn = int(Contact.SOG)

    g_avoid_values, g_avoid_angles_ = Determine_angle(Ownship,Contact,r_cpa_max,r_cpa_min,g_max,g_min) #, r_tcpa,  k_0_vector, k_1_vector, k_2_vector

    h_angles = np.matrix([[0.00 for x in range(7)] for y in range(360)])
    f_function_final = np.matrix([[0.00 for x in range(7)] for y in range(360)])
    h_angles_correct = np.matrix([[0.00 for x in range(7)] for y in range(360)])
    f_function_correct = np.matrix([[0.00 for x in range(7)] for y in range(360)])

    for k in range(7):
        for i in range(360):
            teta_os += 1
            teta_os = HeadingRangeConvert360(teta_os)
            pre_beta = Bearing(Ownship,Contact)
            beta = alpha_and_beta(pre_beta,teta_os)
            pre_alpha = Bearing(Contact,Ownship)
            alpha = alpha_and_beta(pre_alpha,Contact.COG)

            teta_tn = tangent_heading(pre_beta)
            teta_os_tn = heading_deviation(teta_os,teta_tn)
            teta_cn_tn = heading_deviation(Contact.COG,teta_tn)
            v_os_tn = speed_in_tang_head(teta_os_tn,v_os)
            v_cn_tn = speed_in_tang_head(teta_cn_tn,Contact.SOG)
            beta_rate = Bearing_rate(v_os_tn,v_cn_tn,r)

            fore_os_cn__ = fore_os_cn(alpha)
            aft_os_cn__ = aft_os_cn(alpha)
            teta_eps,v_cnh,v_eps,r_eps = Passing_relations_function(Ownship,Contact,fore_os_cn__,pre_beta,r,teta_os,v_os)
            pass_cn, pass_cnp, pass_cns = OwnshipPassingContact(v_eps,aft_os_cn__,beta_rate,fore_os_cn__)

            port_os_cn_ = port_os_cn(alpha)
            star_os_cn_ = star_os_cn(alpha)
            port_cn_os_ = port_cn_os(beta)
            star_cn_os_ = star_cn_os(beta)
            teta_gamma_os, v_gamma_os, r_gamma = Crossing_relations_function(Ownship,Contact,port_os_cn_,pre_beta,r,teta_os,v_os)
            Cross_xcn, Cross_xcnb, Cross_xcns = OwnshipCrossingContact(alpha,v_gamma_os,port_os_cn_,star_os_cn_,beta_rate)
            turn_port,turn_star = turn_func(teta_init,teta_os)

            if (mode == 2):
                Heuristic = Heuristic_Overtaking(submode,pass_cnp,pass_cns)
            elif (mode == 3):
                Heuristic = Heuristic_Head_On(pass_cnp)
            elif (mode == 5):
                Heuristic = Heuristic_Crossing(submode,star_os_cn_,Cross_xcnb,port_os_cn_,turn_port,turn_star) ## FIXME: Not behaving correctly
            else:
                Heuristic = 0
                print """
                    MAJOR ERROR IN HEURISTICS!!!
                """

            h_angles[i,k] = Heuristic
            f_function_final[i,k] = g_avoid_angles_[i,k] * h_angles[i,k] #append row to matrix
        v_os += 0.5

    for m in range(7):
        for j in range(360):
            if j == 0:
                teta_os = 360 - teta_init
                teta_os = HeadingRangeConvert360(teta_os)
                teta_os = int(teta_os)
            else:
                teta_os += 1
                teta_os = HeadingRangeConvert360(teta_os)
                teta_os = int(teta_os)

            f_function_correct[j,m] = f_function_final[teta_os,m]

    f_d,teta_d,v_d = Maximize_f(teta_init,v_init,f_function_correct,mode,v_cn) #Function that loops the f_function to get the desired values

    return h_angles_correct,f_function_correct, teta_d, f_d, v_d#, g_avoid_values #beta_vector_correct,fore_os_cn__vector_correct,v_eps_vector_correct,pass_cn_vector_correct,pass_cnp_vector_correct,

#------CPA mode-----#

def CPA_mode(Ownship,Contact,r_cpa_max,r_cpa_min,g_max,g_min,r,mode,submode):
    #When mode == 1 (CPA), which means no other mode is defined, only g function is apllied with no Heuristic

    teta_init = Ownship.COG
    v_init = Ownship.SOG
    v_os = 0

    v_cn = int(Contact.SOG)

    g_avoid_values, g_avoid_angles_ = Determine_angle(Ownship,Contact,r_cpa_max,r_cpa_min,g_max,g_min) #, r_tcpa,  k_0_vector, k_1_vector, k_2_vector

    h_angles = np.matrix([[0.00 for x in range(7)] for y in range(360)])

    f_function_final = g_avoid_angles_
    f_function_correct = g_avoid_values

    f_d,teta_d,v_d = Maximize_f(teta_init,v_init,f_function_correct,mode,v_cn) #Function that loops the f_function to get the desired values

    return h_angles,f_function_correct, teta_d, f_d, g_avoid_values,v_d #beta_vector_correct,fore_os_cn__vector_correct,v_eps_vector_correct,pass_cn_vector_correct,pass_cnp_vector_correct,

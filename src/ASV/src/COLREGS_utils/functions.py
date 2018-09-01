import rospy
import math
import numpy as np

###-------------- Convert Headings -----------------###
def HeadingRangeConvert360(teta):
    if teta >= 0:
      new_teta = teta - math.floor(teta / 360) * 360
    else:
      new_teta = teta + (math.floor(- teta / 360) + 1) * 360
    return new_teta

def HeadingRangeConvert180(teta):
    if teta >= 0:
      new_teta = teta - math.floor((teta + 180) / 360) * 360
    else:
      new_teta = teta + (math.floor(( - teta + 180 ) / 360)) * 360
    return new_teta

def heading_deviation(teta_a,teta_b):
    teta_dev = abs(HeadingRangeConvert180(( teta_a - teta_b )))
    return teta_dev

def delta_teta(teta,teta_orig):
    delta_teta = HeadingRangeConvert180(teta - teta_orig)
    return delta_teta

###------------ Primary functions -------------##
def range_func(Ownship,Contact): #range between vehicles
    r_os_cn = math.sqrt( (Ownship.x - Contact.x) ** 2 + (Ownship.y - Contact.y) ** 2 )
    return r_os_cn

def Bearing(V_1,V_2): #Bearing from ownship to contact or in reverse
    if (V_1.x == V_2.x and V_1.y <= V_2.y):
        bng_os_cn = 0
    elif (V_1.x == V_2.x and V_1.y > V_2.y):
        bng_os_cn = 180
    elif (V_1.x < V_2.x and V_1.y <= V_2.y):
        bng_os_cn = 90 - math.atan(  abs(V_1.y-V_2.y) / abs(V_1.x - V_2.x) ) * 180 / math.pi
    elif (V_1.x < V_2.x and V_1.y > V_2.y):
        bng_os_cn = math.atan(  abs(V_1.y-V_2.y) / abs(V_1.x - V_2.x) ) * 180 / math.pi + 90
    elif (V_1.x > V_2.x and V_1.y > V_2.y):
        bng_os_cn = 270 - math.atan(  abs(V_1.y-V_2.y) / abs(V_1.x - V_2.x) ) * 180 / math.pi
    elif (V_1.x > V_2.x and V_1.y <= V_2.y):
        bng_os_cn = math.atan(  abs(V_1.y-V_2.y) / abs(V_1.x - V_2.x) ) * 180 / math.pi + 270
    return bng_os_cn

def alpha_and_beta(bng,teta): #function to obtain alpha or beta
    alpha_or_beta = HeadingRangeConvert360(bng - teta)
    return alpha_or_beta

#-------- Range Rate and Bearing Rate ---------#

def vel_in_direction(alpha_or_beta,vel):
    v = math.cos(math.radians(alpha_or_beta)) * vel
    return v

def range_rate_function(v_a,v_b):
    range_r = - ( v_a + v_b )
    return range_r

def tangent_heading(bng):
    teta_tn = HeadingRangeConvert360(bng + 90)
    return teta_tn

def speed_in_tang_head(teta,vel):
    velocity = math.cos(math.radians(teta)) * vel
    return velocity

def Bearing_rate(vel_a,vel_b,r):
    beta_r = - (vel_a + vel_b) * 360 / ( 2 * r * math.pi )
    return beta_r

###------------------ Boolean --------------------###
#------------------ Fore and Aft -------------------#

def fore_os_cn(alpha):
    if (alpha > 90 and alpha < 270):
        fore = 0
    else:
        fore = 1
    return fore

def aft_os_cn(alpha):
    if (alpha >= 90 and alpha <= 270):
        aft = 1
    else:
        aft = 0
    return aft

def fore_cn_os(beta):
    if (beta > 90 and beta < 270):
        fore = 0
    else:
        fore = 1
    return fore

def aft_cn_os(beta):
    if (beta >= 90 and beta <= 270):
        aft = 1
    else:
        aft = 0
    return aft

#---------------- Port and Star ------------------#

def port_os_cn(alpha):
    if (alpha >= 0 and alpha <= 180):
        port = 0
    else:
        port = 1
    return port

def star_os_cn(alpha):
    if (alpha >= 0 and alpha <= 180):
        star = 1
    else:
        star = 0
    return star

def port_cn_os(beta):
    if (beta >= 0 and beta <= 180):
        port = 0
    else:
        port = 1
    return port

def star_cn_os(beta):
    if (beta >= 0 and beta <= 180):
        star = 1
    else:
        star = 0
    return star

##------- Boolean Crossing Relationships -------##

def Crossing_relations_function(Ownship,Contact,bool,bng,r,teta_os,v_os): #teta_os changes
    if bool == 1:
        teta_g = HeadingRangeConvert360(Contact.COG + 90)
    else:
        teta_g = HeadingRangeConvert360(Contact.COG - 90)
    v_gamma = math.cos(math.radians( teta_os - teta_g )) * v_os
    r_g = r * math.cos(math.radians(teta_g - bng))
    return teta_g,v_gamma,r_g

def OwnshipCrossingContact(alpha,v_g,port,star,beta_r):
    if (alpha == 0 or alpha == 180):
        Cross_xcn = 1
    elif (v_g > 0):
        Cross_xcn = 1
    else:
        Cross_xcn = 0
    #---
    if (alpha == 0):
        Cross_xcnb = 1
    elif (v_g > 0 and port == 1 and beta_r > 0):
        Cross_xcnb = 1
    elif (v_g > 0 and star == 1 and beta_r < 0):
        Cross_xcnb = 1
    else:
        Cross_xcnb = 0
    #---
    if (alpha == 180):
        Cross_xcns = 1
    elif (v_g > 0 and port == 1 and beta_r < 0):
        Cross_xcns = 1
    elif (v_g > 0 and star == 1 and beta_r > 0):
        Cross_xcns = 1
    else:
        Cross_xcns = 0
    return Cross_xcn, Cross_xcnb, Cross_xcns

def numericalCrossing(r_gamma,v_gamma,v_cnh_os,v_cn,Cross_xcnb,Cross_xcns,r_eps):
    t_gamma_os = r_gamma / v_gamma
    r_eps_xos = t_gamma_os * v_cnh_os
    r_eps_xcn = t_gamma_os * v_cn

    if (Cross_xcnb == 1):
        r_xcnb = r_eps_xos - ( r_eps - r_eps_xcn )
    else:
        r_xcnb = -1

    if (Cross_xcns == 1):
        r_xcns = ( r_eps - r_eps_xcn ) - r_eps_xos
    else:
        r_xcns = -1
    return r_xcnb, r_xcns

##-------------- Boolean Passing Relationships ----------------##

def Passing_relations_function(Ownship, Contact,bool,bng,r,teta_os,v_os):  #teta_os changes
    if bool == 1: #condition changed to zero. Previously was 1
        teta_eps = HeadingRangeConvert360(Contact.COG + 180)
    else:
        teta_eps = HeadingRangeConvert360(Contact.COG)
    v_cnh = math.cos(math.radians( teta_os - teta_eps )) * v_os
    if bool == 1: #It must be 1, confirmed with Head_On situation
        v_eps = Contact.SOG + v_cnh
    else:
        v_eps = v_cnh - Contact.SOG
    r_eps = r * math.cos(math.radians(teta_eps - bng))
    return teta_eps,v_cnh,v_eps,r_eps

def OwnshipPassingContact(v_eps,aft,beta_r,fore):
    if (v_eps > 0):
        pass_cn = 1
    else:
        pass_cn = 0
    #---
    if (aft == 1 and pass_cn == 1 and beta_r > 0):
        pass_cnp = 1
    elif (fore == 1 and pass_cn == 1 and beta_r < 0):
        pass_cnp = 1
    else:
        pass_cnp = 0
    #---
    if (aft == 1 and pass_cn == 1 and beta_r < 0):
        pass_cns = 1
    elif (fore == 1 and pass_cn == 1 and beta_r > 0):
        pass_cns = 1
    else:
        pass_cns = 0
    return pass_cn, pass_cnp, pass_cns

##-------------- CPA parameters -----------------##

def ks(Ownship,Contact,teta_os,v_os):  #teta_os changes throught time
    k_0 = Ownship.y ** 2 - 2 * Ownship.y * Contact.y + Contact.y ** 2 + Ownship.x ** 2 - 2 * Ownship.x * Contact.x + Contact.x ** 2
    k_1 = 2 * math.cos(math.radians(teta_os)) * v_os * Ownship.y - 2 * math.cos(math.radians(teta_os)) * v_os * Contact.y - 2 * Ownship.y * math.cos(math.radians(Contact.COG)) * Contact.SOG + 2 * math.cos(math.radians(Contact.COG)) * Contact.SOG * Contact.y + 2 * math.sin(math.radians(teta_os)) * v_os * Ownship.x - 2 * math.sin(math.radians(teta_os)) * v_os * Contact.x - 2 * Ownship.x * math.sin(math.radians(Contact.COG)) * Contact.SOG + 2 * math.sin(math.radians(Contact.COG)) * Contact.SOG * Contact.x
    k_2 = (math.cos(math.radians(teta_os))) ** 2 * v_os ** 2 - 2 * math.cos(math.radians(teta_os)) * v_os * math.cos(math.radians(Contact.COG)) * Contact.SOG + (math.cos(math.radians(Contact.COG))) ** 2 * Contact.SOG ** 2 + (math.sin(math.radians(teta_os))) ** 2 * v_os ** 2 - 2 * (math.sin(math.radians(teta_os))) * v_os * math.sin(math.radians(Contact.COG)) * Contact.SOG + (math.sin(math.radians(Contact.COG))) ** 2 * Contact.SOG ** 2
    return k_0,k_1,k_2

def CPA(k_0,k_1,k_2,range_rate):
    if (range_rate >= 0):
        t_cpa = 0
    else:
        t_cpa = - k_1 / ( 2 * k_2 )
    r_tcpa = math.sqrt(k_2 * t_cpa ** 2 + k_1 * t_cpa + k_0)
    return t_cpa,r_tcpa

##------------- Turn boolean -------------##

def turn_func(teta_init,teta):
    if (HeadingRangeConvert360(teta - teta_init) <= 180):
        turn_port = False
        turn_star = True
    else:
        turn_port = True
        turn_star = False
    value = HeadingRangeConvert360(teta - teta_init)
    print value, teta, teta_init, turn_port
    return turn_port,turn_star

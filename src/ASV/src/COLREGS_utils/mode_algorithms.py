import rospy
import numpy as np

from COLREGS_utils import functions

HeadingRangeConvert360 = functions.HeadingRangeConvert360
HeadingRangeConvert180 = functions.HeadingRangeConvert180

###--------Algorithm 2----------###
def Algorithm_2(r,r_pwt,mode,submode,alpha,range_rate,beta,pass_cn,pass_cns,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,fi_headon,port_cn_os_,Cross_xcnb,r_cpa_min,r_xcnb,Cross_xosb,start_manouver_boolean):
    # Determining the COLREGS Major Mode - Executed each iteration of the behaviour

    #    modes   |          submodes  |
    #  --------- | -----    --------- | -----
    #     Null   |   0        None    |   0
    #     CPA    |   1        Port    |   1
    #  GiveWayOT |   2      Starboard |   2
    #   HeadOn   |   3        Bow     |   3
    #  StandOnOT |   4       Stern    |   4
    #  GiveWayX  |   5
    #  StandOnX  |   6
    print "###---Algorithm 2- mode entry---###"
    if (mode == 0 or mode == 1 or mode == 2):
        mode,submode = Algorithm_3(r,r_pwt,mode,submode,alpha,range_rate,beta,pass_cn,pass_cns) #CheckModeGiveWayOT()
        print "-> GiveWayOT ","mode =",mode,"and submode =",submode
    if (mode == 0 or mode == 1 or mode == 3):
        #print "In Algorithm_4:",r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,fi_headon

        mode,submode = Algorithm_4(r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,fi_headon) #CheckModeHeadOn()
        print "-> HeadOn ","mode =",mode,"and submode =",submode
    if (mode == 0 or mode == 1 or mode == 4):
        mode,submode = Algorithm_5(r,r_pwt,mode,submode,range_rate,beta,pass_cn,port_cn_os_) #CheckModeStandOnOT())
        print "-> StandOnOT ","mode =",mode,"and submode =",submode
    if (mode == 0 or mode == 1 or mode == 5):
        mode,submode = Algorithm_6(r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,Cross_xcnb,r_cpa_min,r_xcnb) #CheckModeGiveWayX()
        print "-> GiveWayX ","mode =",mode,"and submode =",submode
    if (mode == 0 or mode == 1 or mode == 6):
        mode,submode = Algorithm_7(r,r_pwt,mode,submode,alpha,range_rate,beta,Cross_xosb) #CheckModeStandOnX()
        print "-> StandOnX ","mode =",mode,"and submode =",submode
    if (mode == 0 or mode == 1):
        if start_manouver_boolean == 1:
            mode = 1
            submode = 0
        else:
            mode = 0
            submode = 0

    print ""
    return mode, submode

def Algorithm_3(r,r_pwt,mode,submode,alpha,range_rate,beta,pass_cn,pass_cns):
    # procedure CheckModeGiveWayOT() - Called from within setAvoidMode()
    #print "Algorithm 3 ---> ", "range_r =",range_rate
    #Part 1: Absolute release Check
    if (r > r_pwt):
        mode = 0 # Null
        submode = 0 #None
        return mode, submode

    #Part 2: Check release from GiveWayOT
    if (mode == 2): #mode == 2 is GiveWayOT mode
        print "part2" "alpha", alpha, "and pass_cn", pass_cn, "and range_rate", range_rate
        if (alpha > 337.5 or alpha < 22.5 or range_rate > 0):
            mode = 1 #CPA
            submode = 0 #None
            #print "Alg2: -> 2 ","mode =",mode,"and submode =",submode," Part 2!", "---> range_r =",range_rate,"alpha =",alpha
            return mode, submode

        #Switch submode to starboard
        if (submode == 1 and beta > 180 and beta < 355): #submode == 1 is Port submode
            mode = 2 #GiveWayOT
            submode = 2 #starboard
            return mode, submode

        #Switch submode to Port
        elif (submode == 2 and beta < 5): #submode == 2 is starboard submode
            mode = 2 #GiveWayOT
            submode = 1 #Port
            return mode, submode

        #Retain submode
        else:
            mode = 2 #GiveWayOT
            submode = submode
            return mode, submode

    #Part 3: Check GiveWayOT mode entry
    print "part3:", "alpha", alpha, "and pass_cn", pass_cn, "and pass_cns", pass_cns
    if (alpha > 247.5 or alpha < 112.5 or pass_cn == 0):
        mode = 1 #CPA
        submode = 0 #None
        #print "Alg2: -> 2 ","mode =",mode,"and submode =",submode," Part 3!"
        return mode, submode

    #Part 4: Determine entry submode
    if (pass_cns == 1):
        mode = 2 #GiveWayOT
        submode = 2 #starboard
        return mode, submode  #GiveWayOT entry case A
    else:
        mode = 2 #GiveWayOT
        submode = 1 #Port
        return mode, submode #GiveWayOT entry case B

def Algorithm_4(r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,fi_headon):
    #print "In Algorithm_4:",r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,aft_os_cn_,star_os_cn_,star_cn_os_,fi_headon
    # procedure CheckModeHeadOn() - Called from within setAvoidMode()
    submode = 0 #None
    #print "I entered Algorithm_4 --> HeadOn"
    #Part 1: Absolute release Check
    if (r > (2 * r_pwt)):
        mode = 0 #Null
        return mode, submode
    #print "Algorithm 4 ---> ", "range_r =",range_rate
    #Part 2: Check release from HeadOn
    if (mode == 3): #mode == 3 is HeadOn mode
        if (r > r_cpa_min and range_rate > 0): #HeadOn is complete
            mode = 1 #CPA
            #print "here 1"
            return mode, submode
        elif (aft_os_cn_ == 1): #ownship passed contact
            mode = 1 #CPA
            #print "here 2"
            return mode, submode
        elif (star_os_cn_ == 1 and star_cn_os_ == 1): #Port-Port may be no longer advisable
            beta_gamma = beta - fi_headon
            alpha_gamma = alpha - fi_headon
            #print "here 3"
            if (beta_gamma > (fi_headon/2) or alpha_gamma > (fi_headon/2) or (beta_gamma + alpha_gamma) > (2 * fi_headon / 3)):
                mode = 1 #CPA
                #print "here 4"
                return mode, submode
        else:
            mode = 3 #HeadOn
            return mode, submode # Mode Unchanged
    #Part 3: Check HeadOn mode entry
    if (mode == 0 or mode == 1): #mode == 0 is Null mode and mode == 1 is CPA mode
        #print "I entered Algorithm_3 --> HeadOn ---> part3: alpha =",abs(HeadingRangeConvert180(alpha)),"beta",abs(HeadingRangeConvert180(beta))
        if (abs(HeadingRangeConvert180(alpha)) <= fi_headon and abs(HeadingRangeConvert180(beta)) <= fi_headon ):
            mode = 3 #HeadOn
            return mode, submode
        else:
            mode = 1 #CPA
            return mode, submode

    return mode, submode

def Algorithm_5(r,r_pwt,mode,submode,range_rate,beta,pass_cn,port_cn_os_):
    # procedure CheckModeStandOnOT() - Called from within setAvoidMode()

    #Part 1: Absolute release Check
    if (r > r_pwt):
        mode = 0 #Null
        submode = 0 #None
        return mode, submode

    #Part 2: Check release from StandOnOT
    if (mode == 4): #mode == 4 is StandOnOT mode
        if (range_rate > 0): #StandOnOT is complete
            mode = 1 #CPA
            submode = 0 #None
            return mode, submode
        else:
            mode = 4 #StandOnOT
            submode = submode
            return mode, submode # Mode Unchanged

    #Part 3: StandOnOT entry criteria
    if (beta < 112.5 or beta > 247.5 or pass_cn == 0):
        mode = 1 #CPA
        submode = 0 #None
        return mode, submode

    #Part 4: Determine submode
    if (port_cn_os_ == 1):
        mode = 4 #StandOnOT
        submode = 1 #Port
        return mode, submode
    else:
        mode = 4 #StandOnOT
        submode = 2 #Starboard
        return mode, submode

def Algorithm_6(r,r_pwt,mode,submode,alpha,range_rate,beta,r_cpa_max,Cross_xcnb,r_cpa_min,r_xcnb):
    # procedure CheckModeGiveWayX() - Called from within setAvoidMode()

    #Part 1: Absolute release Check
    if (r > r_pwt):
        mode = 0 #Null
        submode = 0 #None
        return mode, submode

    #Part 2: Check release from GiveWayX
    if (mode == 5): #mode == 5 is GiveWayX mode
        #print "range_rate =", range_rate
        if (r > r_cpa_min and range_rate > 0): #GiveWayX is complete
            mode = 1 #CPA
            submode = 0 #None
            return mode, submode
        elif (alpha <= 180): #Ownship on contact starboard side
            mode = 1 #CPA
            submode = 0 #None
        else:
            mode = 5 #GiveWayX
            submode = submode
            return mode, submode # Mode and submode Unchanged

    #Part 3: GiveWayX entry criteria
    if (alpha < 247.5 or beta > 112.5):
        mode = 1 #CPA
        submode = 0 #None
        return mode, submode

    #Part 4: Determine submode
    if (Cross_xcnb == 1 and r_xcnb > ( (r_cpa_max + r_cpa_min) / 2 )):
        mode = 5 #GiveWayX
        submode = 3 #Bow
        return mode, submode
    else:
        mode = 5 #GiveWayX
        submode = 4 #Stern
        return mode, submode

def Algorithm_7(r,r_pwt,mode,submode,alpha,range_rate,beta,Cross_xosb):
    # procedure CheckModeStandOnX() - Called from within setAvoidMode()

    #Part 1: Absolute release Check
    if (r > r_pwt):
        mode = 0 #Null
        submode = 0 #None
        return mode, submode

    #Part 2: Check release from StandOnX
    if (mode == 6): #mode == 6 is StandOnX mode
        if (range_rate > 0): #StandOnX is complete
            mode = 1 #CPA
            submode = 0 #None
            return mode, submode
        else:
            mode = 6 #StandOnX
            submode = submode
            return mode, submode # Mode and submode Unchanged

    #Part 3: StandOnX entry criteria
    if (alpha > 112.5 or beta < 247.5 or range_rate >= 0):
        mode = 1 #CPA
        submode = 0 #None
        return mode, submode

    #Part 4: Determine submode
    if (Cross_xosb == 1):
        mode = 6 #StandOnX
        submode = 3 #Bow
        return mode, submode
    else:
        mode = 6 #StandOnX
        submode = 4 #Stern
        return mode, submode

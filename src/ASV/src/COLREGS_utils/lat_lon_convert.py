import rospy
import math
import numpy as np

###--------------Latitude and Longitude convertion-------------###
def geo_utm(lat_0,lon_0,lat,lon):
    #*---------------------------------------------------------*#
    #* parameters and functions for UTM<->Lat/Long conversions *#
    #* UTM conversion is based on WGS84 ellipsoid parameters   *#
    #*---------------------------------------------------------*#
    Earth_radius = 6378137.0 #Earth radius in [m]
    flattening = 0.00335281068 #* GRS80 or WGS84 *#
    K_not = 0.9996     #* UTM scale factor *#

    ##-------Reference Location (_0)------##
    #Get Zone location
    if (lon_0 < 0):
        zone_0 = int( ( ( 180 + lon_0 ) /6 ) + 1 )
    else:
        zone_0 = int( ( lon_0 / 6 ) + 31 )

    #----- first compute the necessary geodetic parameters and constants ------#
    lambda_not_0 = math.radians( ( -180.0 + zone_0 * 6.0 ) - 3.0 )
    e_squared_0 = 2.0 * flattening - flattening * flattening
    e_fourth_0 = e_squared_0 * e_squared_0
    e_sixth_0 = e_fourth_0 * e_squared_0
    e_prime_sq_0 = e_squared_0 / ( 1.0 - e_squared_0 )
    sin_phi_0 = math.sin(math.radians(lat_0))
    tan_phi_0 = math.tan(math.radians(lat_0))
    cos_phi_0 = math.cos(math.radians(lat_0))
    N_0 = Earth_radius / math.sqrt( 1.0 - e_squared_0 * sin_phi_0 * sin_phi_0)
    T_0 = tan_phi_0 * tan_phi_0
    C_0 = e_prime_sq_0 * cos_phi_0 * cos_phi_0

    M_0 = Earth_radius * ( ( 1.0 - e_squared_0 * 0.25 - 0.046875 * e_fourth_0  - 0.01953125 * e_sixth_0 ) * math.radians(lat_0) - \
    ( 0.375 * e_squared_0 + 0.09375 * e_fourth_0 + 0.043945313 * e_sixth_0 ) * math.sin( 2.0 * math.radians(lat_0)) + \
    ( 0.05859375 * e_fourth_0 + 0.043945313 * e_sixth_0 ) * math.sin( 4.0 * math.radians(lat_0) ) - \
    ( 0.011393229 * e_sixth_0) * math.sin( 6.0 * math.radians(lat_0)))

    A_0 = ( math.radians(lon_0) - lambda_not_0 ) * cos_phi_0
    A_sq_0 = A_0 * A_0
    A_fourth_0 =  A_sq_0 * A_sq_0

    #------- now compute X and Y -------#
    x_0 = K_not * N_0 * ( A_0 + ( 1.0 - T_0 + C_0 ) * A_sq_0 * A_0 / 6.0 + ( 5.0 - 18.0 * T_0 + T_0 * T_0 + 72.0 * C_0 - 58.0 * e_prime_sq_0 ) * A_fourth_0 * A_0 / 120.0 )

    y_0 = K_not * ( M_0 + N_0 * tan_phi_0 * ( A_sq_0 / 2.0 + ( 5.0 - T_0+ 9.0 * C_0 + 4.0 * C_0 * C_0 ) * A_fourth_0 / 24.0 + ( 61.0 - 58.0 * T_0 + T_0 * T_0 + 600.0 * C_0 \
    - 330.0 * e_prime_sq_0 ) * A_fourth_0 * A_sq_0 / 720.0 ))

    #------- now correct for false easting and northing -------#
    if( lat_0 < 0):
        y_0 +=10000000.0
        x_0 +=500000

    ##-------GPS location------##
    #Get Zone location
    if (lon < 0):
        zone = int( ( ( 180 + lon ) /6 ) + 1 )
    else:
        zone = int( ( lon / 6 ) + 31 )

    #------- first compute the necessary geodetic parameters and constants -------#
    lambda_not = math.radians( ( -180.0 + zone * 6.0 ) - 3.0 )
    e_squared = 2.0 * flattening - flattening * flattening
    e_fourth = e_squared * e_squared
    e_sixth = e_fourth * e_squared
    e_prime_sq = e_squared / ( 1.0 - e_squared )
    sin_phi = math.sin(math.radians(lat))
    tan_phi = math.tan(math.radians(lat))
    cos_phi = math.cos(math.radians(lat))
    N = Earth_radius / math.sqrt( 1.0 - e_squared * sin_phi * sin_phi)
    T = tan_phi * tan_phi
    C = e_prime_sq * cos_phi * cos_phi

    M = Earth_radius * ( ( 1.0 - e_squared * 0.25 - 0.046875 * e_fourth  - 0.01953125 * e_sixth ) * math.radians(lat) - \
    ( 0.375 * e_squared + 0.09375 * e_fourth + 0.043945313 * e_sixth ) * math.sin( 2.0 * math.radians(lat)) + \
    ( 0.05859375 * e_fourth + 0.043945313 * e_sixth ) * math.sin( 4.0 * math.radians(lat) ) - \
    ( 0.011393229 * e_sixth) * math.sin( 6.0 * math.radians(lat)))

    A = ( math.radians(lon) - lambda_not ) * cos_phi
    A_sq = A * A
    A_fourth =  A_sq * A_sq

    #------- now compute X and Y -------#
    x_utm = K_not * N * ( A + ( 1.0 - T + C ) * A_sq * A / 6.0 + ( 5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e_prime_sq ) * A_fourth * A / 120.0 )

    y_utm = K_not * ( M + N * tan_phi * ( A_sq / 2.0 + ( 5.0 - T + 9.0 * C + 4.0 * C * C ) * A_fourth / 24.0 + ( 61.0 - 58.0 * T + T * T + 600.0 * C \
    - 330.0 * e_prime_sq ) * A_fourth * A_sq / 720.0 ))

    #------- now correct for false easting and northing -------#
    if( lat < 0):
        y_utm +=10000000.0
        x_utm +=500000

    ## ------ x and y with Reference to x_0 and y_0 ------##

    x_utm = x_0 - x_utm

    y_utm = y_utm - y_0

    return y_utm,x_utm  #changed because axis in gazebo are flipped, before the return was: x_utm,y_utm

##for debugging##
#x_tos,y_tos = geo_utm(41.1850,-8.6970,41.18445,-8.69635)
#x_tcn,y_tcn = geo_utm(41.1850,-8.6970,41.18441,-8.69631)
#dif_x = x_tos - x_tcn
#dif_y = y_tos - y_tcn
#print dif_x,dif_y

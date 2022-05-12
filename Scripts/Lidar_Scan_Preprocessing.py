#! /usr/bin/env python

import numpy as np
from math import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

Considered_Max_Lidar_Distance = 1.0
Considered_Collision_Distance = 0.14   # LaserScan.range_min = 0.10000000149
Obstacle_Nearby_Distance = 0.45

ZONE_0_LENGTH = 0.4
ZONE_1_LENGTH = 0.7

Lidar_Angle_Max = 359
Lidar_Angle_Min = 0
Zone_Width = 75

# Converting LasecScan msg to array
def LidarScan_to_array(Scan_Msg):

    Lidar_Distances = np.array([])
    Lidar_Angles = np.array([])

    for i in range(len(Scan_Msg.ranges)):
        
        if ( Scan_Msg.ranges[i] > Considered_Max_Lidar_Distance ): #Considering
            distance = Considered_Max_Lidar_Distance

        elif ( Scan_Msg.ranges[i] < Scan_Msg.range_min ):
            distance = Scan_Msg.range_min
            # For real robot - protection
            # if Scan_Msg.ranges[i] < 0.01:
            #     distance = Considered_Max_Lidar_Distance
        else:
            distance = Scan_Msg.ranges[i]

        angle = degrees(i * Scan_Msg.angle_increment)

        Lidar_Distances = np.append(Lidar_Distances, distance)
        Lidar_Angles = np.append(Lidar_Angles, angle)

    # Lidar_Distances in [m], Lidar_Angles in [degrees]
    return ( Lidar_Distances, Lidar_Angles )

# Discretization of lidar scan
def LaserScan_Discretization(state_space, Lidar_Distances):
    x1 = 2 # Zero_to_Left  (no obstacle detected) #two zones x1 = 0 -> Obstacle too close(<0.4) x1 = 1 -> obstacle(>0.4m but <0.7m) 
    x2 = 2 # Zero_to_Right  (no obstacle detected) same as x1 two sub states
    x3 = 3 # Left Zone (no obstacle detected) three sub states -> (zero to left and left is true x3 = 0) or (left and far left x3 = 1) or (zero to left and left and far left x3 = 2)
    x4 = 3 # Right Zone (no obstacle detected)

    # Find the Zero_to_left side lidar values of the vehicle
    Lidar_Zero_to_left = min(Lidar_Distances[(Lidar_Angle_Min):(Lidar_Angle_Min + Zone_Width)])
    if ZONE_1_LENGTH > Lidar_Zero_to_left > ZONE_0_LENGTH:
        x1 = 1 # zone 1
    elif Lidar_Zero_to_left <= ZONE_0_LENGTH:
        x1 = 0 # zone 0

    # Find the Zero_to_right side lidar values of the vehicle
    Lidar_Zero_to_Right = min(Lidar_Distances[(Lidar_Angle_Max - Zone_Width):(Lidar_Angle_Max)])
    if ZONE_1_LENGTH > Lidar_Zero_to_Right > ZONE_0_LENGTH:
        x2 = 1 # zone 1
    elif Lidar_Zero_to_Right <= ZONE_0_LENGTH:
        x2 = 0 # zone 0

    # Detection of object in front of the robot (0 to 25 degrees either side)
    if ( min(Lidar_Distances[(Lidar_Angle_Max - Zone_Width // 3):(Lidar_Angle_Max)]) < 1.0 ) or ( min(Lidar_Distances[(Lidar_Angle_Min):(Lidar_Angle_Min + Zone_Width // 3)]) < 1.0 ):
        Object_Front = True
    else:
        Object_Front = False

    # Detection of object on the left side of the robot 0 to 50 degrees
    if min(Lidar_Distances[(Lidar_Angle_Min):(Lidar_Angle_Min + 2 * Zone_Width // 3)]) < 1.0:
        Object_Left = True
    else:
        Object_Left = False

    # Detection of object on the right side of the robot
    if min(Lidar_Distances[(Lidar_Angle_Max - 2 * Zone_Width // 3):(Lidar_Angle_Max)]) < 1.0:
        Object_Right = True
    else:
        Object_Right = False

    # Detection of object on the far left side of the robot  50 to 75 degrees
    if min(Lidar_Distances[(Lidar_Angle_Min + Zone_Width // 3):(Lidar_Angle_Min + Zone_Width)]) < 1.0:
        Object_Far_Left = True
    else:
        Object_Far_Left = False

    # Detection of object on the far right side of the robot
    if min(Lidar_Distances[(Lidar_Angle_Max - Zone_Width):(Lidar_Angle_Max - Zone_Width // 3)]) < 1.0:
        Object_Far_Right = True
    else:
        Object_Far_Right = False

    # The left sector of the vehicle
    if ( Object_Front and Object_Left ) and ( not Object_Far_Left ):
        x3 = 0 # sector 0
    elif ( Object_Left and Object_Far_Left ) and ( not Object_Front ):
        x3 = 1 # sector 1
    elif Object_Front and Object_Left and Object_Far_Left:
        x3 = 2 # sector 2

    if ( Object_Front and Object_Right ) and ( not Object_Far_Right ):
        x4 = 0 # sector 0
    elif ( Object_Right and Object_Far_Right ) and ( not Object_Front ):
        x4 = 1 # sector 1
    elif Object_Front and Object_Right and Object_Far_Right:
        x4 = 2 # sector 2

    # Find the state space index of (x1,x2,x3,x4) in Q table
    ss = np.where(np.all(state_space == np.array([x1,x2,x3,x4]), axis = 1)) #axis = 1 -> one row all columns
    state_ind = int(ss[0])

    return ( state_ind, x1, x2, x3 , x4 )

# Check - crash
def checkCrash(Lidar_Distances):
    lidar_horizon = np.concatenate((Lidar_Distances[(Lidar_Angle_Min + Zone_Width):(Lidar_Angle_Min):-1],Lidar_Distances[(Lidar_Angle_Max):(Lidar_Angle_Max - Zone_Width):-1]))
    Weighted_array = np.linspace(1.2, 1, len(lidar_horizon) // 2)
    Weighted_array = np.append(Weighted_array, np.linspace(1, 1.2, len(lidar_horizon) // 2))
    if np.min( Weighted_array * lidar_horizon ) < Considered_Collision_Distance:
        return True
    else:
        return False

# Check - object nearby
def checkObjectNearby(Lidar_Distances):
    lidar_horizon = np.concatenate((Lidar_Distances[(Lidar_Angle_Min + Zone_Width):(Lidar_Angle_Min):-1],Lidar_Distances[(Lidar_Angle_Max):(Lidar_Angle_Max - Zone_Width):-1]))
    Weighted_array = np.linspace(1.4, 1, len(lidar_horizon) // 2)
    Weighted_array = np.append(Weighted_array, np.linspace(1, 1.4, len(lidar_horizon) // 2))
    if np.min( Weighted_array * lidar_horizon ) < Obstacle_Nearby_Distance:
        return True
    else:
        return False

# Check - goal near
def checkGoalNear(x, y, x_goal, y_goal):
    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if ro < 0.3:
        return True
    else:
        return False
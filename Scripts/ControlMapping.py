#!/usr/bin/env python

import rospy
from time import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Q-learning speed parameters
Linearspeed_Forward = 0.08
Angularspeed_Forward = 0.0

Linearspeed_Turn = 0.06
Angularspeed_Turn = 0.4

# Get theta in [radians]
def Get_Rotation(Odom_Msg):
    orientation_q = Odom_Msg.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

# Get (x,y) coordinates in [m]
def Get_Position(Odom_Msg):
    x = Odom_Msg.pose.pose.position.x
    y = Odom_Msg.pose.pose.position.y
    return ( x , y)

# Get linear speed in [m/s]
def Get_Linear_Velocity(Odom_Msg):
    return Odom_Msg.twist.twist.linear.x

# Get angular speed in [rad/s] - z axis
def Get_Angular_Velocity(Odom_Msg):
    return Odom_Msg.twist.twist.angular.z

# Create rosmsg Twist()
def createVelMsg(v,w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

# Go forward command
def Go_Forward(velPub):
    velMsg = createVelMsg(Linearspeed_Forward,Angularspeed_Forward)
    velPub.publish(velMsg)

# Turn left command
def Go_Left(velPub):
    velMsg = createVelMsg(Linearspeed_Turn,+Angularspeed_Turn)
    velPub.publish(velMsg)

# Turn right command
def Go_Right(velPub):
    velMsg = createVelMsg(Linearspeed_Turn,-Angularspeed_Turn)
    velPub.publish(velMsg)

# Stop command
def Stop_robot(velPub):
    velMsg = createVelMsg(0.0,0.0)
    velPub.publish(velMsg)

# Set robot position and orientation
def Set_robot_Position(setPosPub, x, y, theta):
    checkpoint = ModelState()

    checkpoint.model_name = 'turtlebot3_burger'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return ( x , y , theta )

# Set random initial robot position and orientation
def Set_Random_Robot_Position(setPosPub):
    x_range = np.array([-0.4, 0.6, 0.6, -1.4, -1.4, 2.0, 2.0, -2.5, 1.0, -1.0])
    y_range = np.array([-0.4, 0.6, -1.4, 0.6, -1.4, 1.0, -1.0, 0.0, 2.0, 2.0])
    theta_range = np.arange(0, 360, 15)
    #theta_range = np.array([0, 30, 45, 60, 75, 90])

    ind = np.random.randint(0,len(x_range))
    ind_theta = np.random.randint(0,len(theta_range))

    x = x_range[ind]
    y = y_range[ind]
    theta = theta_range[ind_theta]

    checkpoint = ModelState()

    checkpoint.model_name = 'turtlebot3_burger'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return ( x , y , theta )

# Perform an action
def robotDoAction(velPub, action):
    status = 'robotDoAction => OK'
    if action == 0:
        Go_Forward(velPub)
    elif action == 1:
        Go_Left(velPub)
    elif action == 2:
        Go_Right(velPub)
    else:
        status = 'robotDoAction => INVALID ACTION'
        Go_Forward(velPub)

    return status

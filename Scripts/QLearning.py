#! /usr/bin/env python

import numpy as np
from math import *
from std_msgs.msg import String
from itertools import product
from sensor_msgs.msg import LaserScan

State_Space_Max_Idx = 143
State_Space_Min_Idx = 0

Action_Max_Idx = 2
Action_Min_Idx = 0

Max_Angle = 359
Min_Angle = 0
Zone_Width = 75

# T_MIN = 0.001

# Create actions
def createActions():
    actions = np.array([0,1,2])
    return actions

# Create state space for Q table
def createStateSpace():
    x1 = set((0,1,2))
    x2 = set((0,1,2))
    x3 = set((0,1,2,3))
    x4 = set((0,1,2,3))
    state_space = set(product(x1,x2,x3,x4))
    return np.array(list(state_space))

# Create Q table, dim: n_states x n_actions
def createQTable(n_states, n_actions):
    Q_table = np.zeros((n_states, n_actions))
    return Q_table

# Read Q table from path
def readQTable(path):
    Q_table = np.genfromtxt(path, delimiter = ' , ')
    return Q_table

# Write Q table to path
def saveQTable(path, Q_table):
    np.savetxt(path, Q_table, delimiter = ' , ')

# Select the best action a in state
def getBestAction(Q_table, state_ind, actions):
    if State_Space_Min_Idx <= state_ind <= State_Space_Max_Idx:
        Max_ActionValue_Idx = np.argmax(Q_table[state_ind,:])
        status = 'getBestAction => OK'
        a = actions[Max_ActionValue_Idx]
    else:
        status = 'getBestAction => INVALID STATE INDEX'
        a = getRandomAction(actions)

    return ( a, status )

# Select random action from actions
def getRandomAction(actions):
    n_actions = len(actions)
    Max_ActionValue_Idx = np.random.randint(n_actions)
    return actions[Max_ActionValue_Idx]

# Epsilog Greedy Exploration action chose
def epsiloGreedyExploration(Q_table, state_ind, actions, epsilon):
    if np.random.uniform() > epsilon and State_Space_Min_Idx <= state_ind <= State_Space_Max_Idx:
        status = 'epsiloGreedyExploration => OK'
        ( a, Status_getBestAction ) = getBestAction(Q_table, state_ind, actions)
        if Status_getBestAction == 'getBestAction => INVALID STATE INDEX':
            status = 'epsiloGreedyExploration => INVALID STATE INDEX'
    else:
        status = 'epsiloGreedyExploration => OK'
        a = getRandomAction(actions)

    return ( a, status )

# Reward function for Q-learning - table
def getReward(action, prev_action, lidar, prev_lidar, crash):
    if crash:
        terminal_state = True
        Total_Reward = -100
    else:
        lidar_horizon = np.concatenate((lidar[(Min_Angle + Zone_Width):(Min_Angle):-1],lidar[(Max_Angle):(Max_Angle - Zone_Width):-1]))
        prev_lidar_horizon = np.concatenate((prev_lidar[(Min_Angle + Zone_Width):(Min_Angle):-1],prev_lidar[(Max_Angle):(Max_Angle - Zone_Width):-1]))
        terminal_state = False
        # Reward from action taken = fowrad -> +0.2 , turn -> -0.1
        if action == 0:
            Reward_Action = +0.2
        else:
            Reward_Action = -0.1
        # Reward from crash distance to obstacle change
        Weighted_array = np.linspace(0.9, 1.1, len(lidar_horizon) // 2)
        Weighted_array = np.append(Weighted_array, np.linspace(1.1, 0.9, len(lidar_horizon) // 2))

        if np.sum( Weighted_array * ( lidar_horizon - prev_lidar_horizon) ) >= 0:
            Reward_Obstacle = +0.2
        else:
            Reward_Obstacle = -0.2
        # Reward from turn left/right change
        if ( prev_action == 1 and action == 2 ) or ( prev_action == 2 and action == 1 ):
            Reward_Turn = -0.8
        else:
            Reward_Turn = 0.0

        # Cumulative reward
        Total_Reward = Reward_Action + Reward_Obstacle + Reward_Turn

    return ( Total_Reward, terminal_state )

# Update Q-table values
def updateQTable(Q_table, state_ind, action, reward, next_state_ind, alpha, gamma):
    if State_Space_Min_Idx <= state_ind <= State_Space_Max_Idx and State_Space_Min_Idx <= next_state_ind <= State_Space_Max_Idx:
        status = 'updateQTable => OK'
        Q_table[state_ind,action] = ( 1 - alpha ) * Q_table[state_ind,action] + alpha * ( reward + gamma * max(Q_table[next_state_ind,:]) )
    else:
        status = 'updateQTable => INVALID STATE INDEX'
    return ( Q_table, status )

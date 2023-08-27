#!/usr/bin/env python

import os
import argparse
import copy
from pickle import TRUE
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

class Parameters:
    SPIN_RATE = 20
    home = np.radians([120, -90, 90, -90, -90, 0])
    Q = None
    vel_grip = 1.0
    acc_grip = 1.0
    vel_arm = 4.0
    acc_arm = 4.0
    position_tolerance = 0.0005

class RobotStates:
    current_position = np.radians([120, -90, 90, -90, -90, 0])
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    digital_in_0 = 0
    analog_in_0 = 0
    analog_in_1 = 0
    suction_on = True
    suction_off = False
    current_io_0 = False
    current_position_set = False

def gripper_callback(msg, robot_states):
    robot_states.digital_in_0 = msg.DIGIN
    robot_states.analog_in_0 = msg.AIN0
    robot_states.analog_in_1 = msg.AIN1


def position_callback(msg, robot_states):
    robot_states.thetas[0] = msg.position[0]
    robot_states.thetas[1] = msg.position[1]
    robot_states.thetas[2] = msg.position[2]
    robot_states.thetas[3] = msg.position[3]
    robot_states.thetas[4] = msg.position[4]
    robot_states.thetas[5] = msg.position[5]

    robot_states.current_position[0] = robot_states.thetas[0]
    robot_states.current_position[1] = robot_states.thetas[1]
    robot_states.current_position[2] = robot_states.thetas[2]
    robot_states.current_position[3] = robot_states.thetas[3]
    robot_states.current_position[4] = robot_states.thetas[4]
    robot_states.current_position[5] = robot_states.thetas[5]

    robot_states.current_position_set = True


def gripper(pub_cmd, loop_rate, io_0, robot_states, params):

    error = 0
    spin_count = 0
    at_goal = 0

    robot_states.current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = robot_states.current_position
    driver_msg.v = params.vel_grip
    driver_msg.a = params.acc_grip
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(robot_states.thetas[0]-driver_msg.destination[0]) < params.position_tolerance and \
            abs(robot_states.thetas[1]-driver_msg.destination[1]) < params.position_tolerance and \
            abs(robot_states.thetas[2]-driver_msg.destination[2]) < params.position_tolerance and \
            abs(robot_states.thetas[3]-driver_msg.destination[3]) < params.position_tolerance and \
            abs(robot_states.thetas[4]-driver_msg.destination[4]) < params.position_tolerance and \
            abs(robot_states.thetas[5]-driver_msg.destination[5]) < params.position_tolerance ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count > params.SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_arm(pub_cmd, loop_rate, dest, robot_states, params):
    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = params.vel_arm
    driver_msg.a = params.acc_arm
    driver_msg.io_0 = robot_states.current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(robot_states.thetas[0]-driver_msg.destination[0]) < params.position_tolerance and \
            abs(robot_states.thetas[1]-driver_msg.destination[1]) < params.position_tolerance and \
            abs(robot_states.thetas[2]-driver_msg.destination[2]) < params.position_tolerance and \
            abs(robot_states.thetas[3]-driver_msg.destination[3]) < params.position_tolerance and \
            abs(robot_states.thetas[4]-driver_msg.destination[4]) < params.position_tolerance and \
            abs(robot_states.thetas[5]-driver_msg.destination[5]) < params.position_tolerance ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  params.SPIN_RATE*5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1
    return error


def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height, robot_states, params):
    # Definition of our tower - 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

    # First index - From left to right position A, B, C
    # Second index - From "bottom" to "top" position 1, 2, 3
    # Third index - From gripper contact point to "in the air" point

    Q = params.Q

    # How the arm will move (Suggestions)
    # 1. Go to the "above (start) block" position from its base position
    # 2. Drop to the "contact (start) block" position
    # 3. Rise back to the "above (start) block" position
    # 4. Move to the destination "above (end) block" position
    # 5. Drop to the corresponding "contact (end) block" position
    # 6. Rise back to the "above (end) block" position

    ##### Your Code Starts Here #####
    
    # TODO: Complete the steps to move one block from one 
    #       location to another, using the Q matrix data
    # Hint: 
    # * use move_arm(), gripper() functions
    # * use the 'robot_states.digital_in_0' value to check if a block is detected
    # * use 'sys.exit()' to quit the program
    
    # if robot does not have anything gripped
    if(robot_states.digital_in_0 == False):
        
        #1
        move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], robot_states, params)
        #2
        move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][0], robot_states, params)
        #actuate grip
        gripper(pub_cmd, loop_rate, True, robot_states, params)

        #if something is gripped
        if(robot_states.digital_in_0 == True):

            #3
            move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], robot_states, params)
            #4
            move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], robot_states, params)
            #5
            move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][0], robot_states, params)
            #release block
            gripper(pub_cmd, loop_rate, False, robot_states, params)
            #6
            move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], robot_states, params)

        #if nothing is gripped
        else:
            print("Nothing is at the starting tower")
            sys.exit()

    #if robot has something gripped already 
    else:      
        print("Something is already gripped")
        sys.exit()
    ##### Your Code Ends Here #####


def get_tower_index():
    input_start = raw_input("Enter start tower <Either 1 2 3 or 0 to quit> ")
    start_tower = 0 
    input_dest = raw_input("Enter destination tower <Either 1 2 3 or 0 to quit> ")
    destination_tower = 0

    if(int(input_start) == 1):
        start_tower = 1
    elif (int(input_start) == 2):
        start_tower = 2
    elif (int(input_start) == 3):
        start_tower = 3
    else:
        start_tower = 0

    ##### Your Code Starts Here #####
    # TODO: get input for destination_tower

    # if input is one of the valid numbers, then assign the destination tower to that number
    if(int(input_dest) == 1):
        destination_tower = 1
    elif (int(input_dest) == 2):
        destination_tower = 2
    elif (int(input_dest) == 3):
        destination_tower = 3
    #if input is not one of the valid numbers, set it to 0, which prompts the script to exit afterwards
    else:
        destination_tower = 0

    ##### Your Code Ends Here #####

    return start_tower, destination_tower
        

def main():
    # Initialize the parameters and robot states (using class)
    params = Parameters()
    robot_states = RobotStates()

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2')
    yamlpath = os.path.join(lab2_path, 'config', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            data = yaml.load(f)
            params.Q = data['sim_pos']
        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('lab2_node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback, (robot_states))
    sub_grip = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback, (robot_states))

    input_done = False
    while(not input_done):
        start_tower, destination_tower = get_tower_index()

        ##### Your Code Starts Here #####
        # TODO: check if the indices are valid
        # - if either index is 0, quit the system
        if(start_tower == 0 | destination_tower == 0):
            print("Invalid input")
            sys.exit()
        # - if starting and destination are the same, ask to re-enter the numbers
        # - stay in the while loop and wait for input again
        elif(start_tower == destination_tower):
            print("Start and destination towers cannot be the same.\nRe-enter the numbers")
            input_done = False
        # - if both are valid, jump out of this while loop to continue
        else:
            input_done = True
        ##### Your Code Ends Here #####
        

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(params.SPIN_RATE)
    
    if (start_tower == 1 and destination_tower == 3) or (start_tower == 3 and destination_tower == 1):
        aux_tower = 2
    elif (start_tower == 2 and destination_tower == 3) or (start_tower == 3 and destination_tower == 2):
        aux_tower = 1
    elif (start_tower == 1 and destination_tower == 2) or (start_tower == 2 and destination_tower == 1):
        aux_tower = 3

    ############## Your Code Start Here ##############
    
    # TODO: using 'move_block' function defined above to guide UR3 
    #       to move tower accordingly from user input
    move_block(pub_command, loop_rate, start_tower-1, 2, destination_tower-1, 0, robot_states, params)
    move_block(pub_command, loop_rate, start_tower-1, 1, aux_tower-1, 0, robot_states, params)
    move_block(pub_command, loop_rate, destination_tower-1, 0, aux_tower-1, 1, robot_states, params)
    move_block(pub_command, loop_rate, start_tower-1, 0, destination_tower-1, 0, robot_states, params)
    move_block(pub_command, loop_rate, aux_tower-1, 1, start_tower-1, 0, robot_states, params)
    move_block(pub_command, loop_rate, aux_tower-1, 0, destination_tower-1, 1, robot_states, params)
    move_block(pub_command, loop_rate, start_tower-1, 0, destination_tower-1, 2, robot_states, params)
    ############### Your Code End Here ###############


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

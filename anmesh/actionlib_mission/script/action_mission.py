#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib


import actionlib_mission.msg

def mission_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('depth', actionlib_mission.msg.missionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_mission.msg.misionGoal(order=10)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def main ():
    rospy.init_node('mission_client_py')
    result = mission_client()
    print(result)
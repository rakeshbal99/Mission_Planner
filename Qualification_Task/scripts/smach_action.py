#!/usr/bin/env python

import roslib; roslib.load_manifest('actionlib_mission')
import rospy
import smach
import smach_ros
import actionlib


import actionlib_mission.msg
# define state Depth
class Depth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_D','FAIL_D']) #Here FAIL_D is the Failure of Depth State

    def execute(self, userdata):
        rospy.loginfo('Executing state DEPTH')

     
        client = actionlib.SimpleActionClient('depth', actionlib_mission.msg.missionAction)

        client.wait_for_server()

    
        goal = actionlib_mission.msg.missionGoal()
        goal.depth=10
   
        client.send_goal(goal)

    
        client.wait_for_result()
        if client.get_result():
            return 'SUCCESS_D'
        else:
            return 'FAIL_D'

# define state Translate
class Translate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_T','FAIL_T']) #Here FAIL_T is the Failure of Translate State

    def execute(self, userdata):
        rospy.loginfo('Executing state TRANSLATE')
        # if self.distance == 10 and rotate==false and self.depth < d1 and self.depth>d2 :
        #     return 'reached_10'
        # if self.distance == 10 and rotate==true and self.depth < d1 and self.depth>d2 :
        #     return 'reached_20'
        # else :
        #     return 'FAIL_T'
        client = actionlib.SimpleActionClient('translate', actionlib_mission.msg.missionAction)

        client.wait_for_server()

    
        goal = actionlib_mission.msg.missionGoal()
        goal.translate=10
        goal.depth=10
   
        client.send_goal(goal)

    
        client.wait_for_result()
        if client.get_result():
            return 'SUCCESS_T'
        else:
            return 'FAIL_T'
    
        

        # define state Rotate
class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_R','FAIL_R'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ROTATE')
        client = actionlib.SimpleActionClient('Rotate', actionlib_mission.msg.missionAction)

        client.wait_for_server()

    
        goal = actionlib_mission.msg.missionGoal()
        goal.rotate=1
        goal.depth=10
   
        client.send_goal(goal)

    
        client.wait_for_result()
        if client.get_result():
            return 'SUCCESS_R'
        else:
            return 'FAIL_R'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Depth', Depth(), 
                               transitions={'SUCCESS_D':'Translate', 
                                            'FAIL_D':'outcome5'})
        smach.StateMachine.add('Translate', Translate(), 
                               transitions={'SUCCESS_T':'Rotate', 
                                            'FAIL_T':'outcome5'})
        smach.StateMachine.add('Rotate', Rotate(), 
                               transitions={'SUCCESS_R':'Depth', 
                                            'FAIL_R':'outcome5'})

    # Execute SMACH plan
    outcome = sm.execute()

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

# Execute the state machine
    outcome = sm.execute()

# Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
#!/usr/bin/env python

import roslib; roslib.load_manifest('demo')
import rospy
import smach
import smach_ros

# define state Depth
class Depth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_D','FAIL_D']) #Here FAIL_D is the Failure of Depth State

    def execute(self, userdata):
        rospy.loginfo('Executing state DEPTH')
        if self.depth < d1 and self.depth>d2 :#Here d1 and d2 are the lower and upper limits of the required depths
            return 'SUCCESS_D'
        else:
            return 'FAIL_D'


# define state Translate
class Translate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached_10','reached_20','FAIL_T']) #Here FAIL_T is the Failure of Translate State

    def execute(self, userdata):
        rospy.loginfo('Executing state TRANSLATE')
        if self.distance == 10 and rotate==false and self.depth < d1 and self.depth>d2 :
            return 'reached_10'
        if self.distance == 10 and rotate==true and self.depth < d1 and self.depth>d2 :
            return 'reached_20'
        else :
            return 'FAIL_T'

# define state Rotate
class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_R','FAIL_R'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ROTATE')
        if self.rotate == true:
            return 'SUCCESS_R'
        else:
            return 'FAIL_R'


def main():
    rospy.init_node('mision_planner_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCESS'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('DEPTH', Depth(),
                               transitions={'SUCCESS_D':'SUB', 
                                            'FAIL_D':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['SUCCESS_M'])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('TRANSLATE', Translate(), 
                                   transitions={'reached_10':'ROTATE', 
                                                'reached_20':'SUCCESS_M',
                                                'FAIL_T' : 'DEPTH'})
            smach.StateMachine.add('ROTATE', Rotate(), 
                                   transitions={'SUCCESS_R':'TRANSLATE',
                                                'FAIL_R':'TRANSLATE'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'SUCCESS_M':'SUCCESS'})

    # Execute SMACH plan
    outcome = sm_top.execute()



if __name__ == '__main__':
    main()
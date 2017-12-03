import roslib; roslib.load_manifest('actionlib_mission')
import rospy
import smach
import smach_ros
import actionlib


import actionlib_mission.msg
# define state Depth
#copied from actionlib_mission changes to made after data is known
class Depth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCESS_D','FAIL_D']) #Here FAIL_D is the Failure of Depth State

    def execute(self, userdata):
        rospy.loginfo('Executing state DEPTH')

     
        client = actionlib.SimpleActionClient('depth', actionlib_mission.msg.missionAction)

        client.wait_for_server()

    
        goal = actionlib_mission.msg.missionGoal()
        goal.depth=.5
   
        client.send_goal(goal)

    
        client.wait_for_result()
        if client.get_result():
            return 'SUCCESS_D'
        else:
            return 'FAIL_D'

#copied from missionplanner_niot changes to made after data is known

class IpSearch(smach.State):
	def __init__(self,resources):
		smach.State.__init__(self, outcomes=['found','time_out'])
		self.resources=resources
	def execute(self, ud):



		#do necessary stuff for moving in some path
		client=self.resources.advancedControllerClient

		# change the below line a/c to the message header
		goal= advancedControllerGoal()

		goal.depth=self.resources.depth
		goal.x=self.resources.pose[header.PoseEnum.x]
		goal.y=-0.1
		client.send_goal_and_wait(goal)
		#do necessary stuff for moving in some path
		return 'found'

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





class Yaw(smach.State):
	def __init__(self):






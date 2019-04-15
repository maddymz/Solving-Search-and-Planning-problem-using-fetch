#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Move base connected")

    def goto(self, x, y, quat, frame="map"):
        global pose
        print("going to ", x , y, quat)
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
    
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        if(quat[3] == 0):
            move_goal.target_pose.pose.orientation.w = 1
        else:
            move_goal.target_pose.pose.orientation.w = quat[3]
        
        # move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        # move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        
        self.client.send_goal(move_goal)
        print("goal sent")
        self.client.wait_for_result()
        print("result done")

class moveFetch:
	def __init__(self):
		rospy.init_node('move_turtle',anonymous = True)
		self.move_base = MoveBaseClient()
		self.actions = String()
		self.robot_pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.action_callback)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.free = String(data = "next")
		self.robot_x = 0
		self.robot_y = 0
		self.robot_orient = "EAST"
		self.rate = rospy.Rate(30)
		print("Ready")
		rospy.spin()

	def pose_callback(self,data):
		self.robot_pose = data.pose.pose

	def get_new_location(self, x, y, orient, action):
		quat = (self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
		if action == "MoveF":
			if orient == "EAST":
				x+=1
			elif orient == "WEST":
				x-=1
			elif orient == "SOUTH":
				y-=1
			else:
				y+=1
		elif action == "MoveB":
			if orient == "EAST":
				x-=1
			elif orient == "WEST":
				x+=1
			elif orient == "SOUTH":
				y+=1
			else:
				y-=1
		elif action == "TurnCW":
			euler = tf.transformations.euler_from_quaternion(quat)
			quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] - math.pi/2.0)
			if orient == "EAST":
				orient = "SOUTH"
			elif orient == "WEST":
				orient = "NORTH"
			elif orient == "SOUTH":
				orient = "WEST"
			else:
				orient = "EAST"
		elif action == "TurnCCW":
			euler = tf.transformations.euler_from_quaternion(quat)
			quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + math.pi/2.0)
			if orient == "EAST":
				orient = "NORTH"
			elif orient == "WEST":
				orient = "SOUTH"
			elif orient == "SOUTH":
				orient = "EAST"
			else:
				orient = "WEST"
		return x, y, quat, orient

	def action_callback(self, data):
		# global robot_x, robot_y, robot_orient
		actions = data.data.split('_')
		print(actions)
		for action in actions: 
			print(action)
			self.robot_x, self.robot_y, quat, self.robot_orient = self.get_new_location(self.robot_x, self.robot_y,self.robot_orient, action)
			if(action == "MoveF"):    
				self.move_base.goto(self.robot_x, self.robot_y, quat)
			elif(action == "TurnCW"):
				self.move_base.goto(self.robot_x, self.robot_y, quat)
			elif(action == "MoveB"):
				self.move_base.goto(self.robot_x, self.robot_y, quat)
			else:
				self.move_base.goto(self.robot_x, self.robot_y, quat)
			rospy.Rate(1).sleep()

if __name__ == "__main__":
	try:
		moveFetch()
	except rospy.ROSInterruptException:
		pass
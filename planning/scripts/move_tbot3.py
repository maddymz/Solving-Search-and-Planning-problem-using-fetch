#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from pid import PID
import copy

class moveTbot3:
	def __init__(self):
		rospy.init_node('move_turtle',anonymous = True)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
		self.pid_subscriber = rospy.Subscriber("/Controller_Status",String,self.callback_pid)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.free = String(data = "Idle")
		self.rate = rospy.Rate(30)
		rospy.spin()

	def callback_pid(self,data):
		if data.data == "Done":
			if len(self.actions)>0:
				self.execute_next()

	def callback_actions(self,data):
		self.actions = data.data.split("_")
		self.rate.sleep()
		self.execute_next()
		# self.move()

	def execute_next(self):
		action = self.actions.pop(0)
		direction = None
		if action == "MoveF" or action == "MoveB":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]
			if current_yaw > (-math.pi /4.0) and current_yaw < (math.pi / 4.0):
				print "Case 1"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				direction = 'x'
				#incr y co-ordinate
			elif current_yaw > (math.pi / 4.0 ) and current_yaw < (3.0 * math.pi / 4.0):
				print "Case 2"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += 0.5
				direction = 'y'
				#decr x co
			elif current_yaw > (-3.0*math.pi /4.0) or current_yaw < (-math.pi /4.0):
				print "Case 3"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				print target_pose
				target_pose.position.y -= 0.5
				print target_pose
				direction = '-y'
			else:
				print "Case 4"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				direction = 'x'
			PID(target_pose,"linear", direction).publish_velocity()
			
		elif action == "TurnCW" or action == "TurnCCW":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			if action == "TurnCW":
				target_yaw = yaw - ( math.pi / 2.0)
				if target_yaw < -math.pi:
					target_yaw += (math.pi * 2)
			else:
				target_yaw = yaw + ( math.pi / 2.0)
				if target_yaw >= (math.pi ):
					target_yaw -= (math.pi * 2 )
			target_pose = Pose()
			target_pose.position = current_pose.position
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(euler[0],euler[1],target_yaw))
			target_pose.orientation = target_quat
			PID(target_pose,"rotational",'None').publish_velocity()

		else:
			print "Invalid action"
			exit(-1)
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose

	# def move(self):
	# 	vel_msg = Twist()
	# 	for action in self.actions.split("_"):
	# 		init_pose = self.pose
	# 		quat = (init_pose.orientation.x,init_pose.orientation.y,init_pose.orientation.z,init_pose.orientation.w)
	# 		init_euler = tf.transformations.euler_from_quaternion(quat)
	# 		yaw = init_euler[2]
	# 		print "Current Yaw :",yaw
	# 		position = init_pose.position


	# 		vel_msg.linear.x = 0
	# 		vel_msg.linear.y = 0
	# 		vel_msg.linear.z = 0
	# 		vel_msg.angular.x = 0
	# 		vel_msg.angular.y = 0
	# 		vel_msg.angular.z = 0
		
	# 		if action == "MoveF" or action == "MoveB":
	# 			speed = 0.05
	# 			if action == "MoveF":
	# 				vel_msg.linear.x = speed
	# 			else:
	# 				vel_msg.linear.x = -speed
	# 			t0 = rospy.Time.now().to_sec()
	# 			distance = 0.5
	# 			current_distance = 0
	# 			while current_distance < distance:
	# 				self.vel_pub.publish(vel_msg)
	# 				t1 = rospy.Time().now().to_sec()
	# 				current_distance = speed * (t1-t0)
	# 			vel_msg.linear.x = 0
	# 			vel_msg.angular.z = 0
	# 			self.vel_pub.publish(vel_msg)
	# 		elif action == "TurnCW" or action == "TurnCCW":
	# 			speed = 0.15
	# 			if action == "TurnCW":
	# 				vel_msg.angular.z = -speed
	# 				target_yaw = yaw - ( math.pi / 2.0)
	# 				if target_yaw < -math.pi:
	# 					target_yaw += (math.pi * 2)
	# 			else:
	# 				vel_msg.angular.z = speed
	# 				target_yaw = yaw + ( math.pi / 2.0)
	# 				if target_yaw >= (math.pi ):
	# 					target_yaw -= (math.pi * 2 )
	# 			print "Target_yaw:",target_yaw
	# 			quat = (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w)
	# 			euler = tf.transformations.euler_from_quaternion(quat)
	# 			current_yaw = euler[2]
	# 			while abs(current_yaw - target_yaw) > 0.1e-2:
	# 				if action == "TurnCW":
	# 					vel_msg.angular.z = -min(speed,min(speed*1.5,abs(current_yaw - target_yaw)/1.5))
	# 				elif action == "TurnCCW":
	# 					vel_msg.angular.z = min(speed,min(speed*1.5,abs(current_yaw - target_yaw)/1.5))
	# 				self.vel_pub.publish(vel_msg)
	# 				quat = (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w)
	# 				euler = tf.
	# 				.euler_from_quaternion(quat)
	# 				current_yaw = euler[2]
	# 			vel_msg.angular.z = 0
	# 			self.vel_pub.publish(vel_msg)
	# 			rospy.sleep(5)
	# 			model_state = ModelState()
	# 			model_state.model_name = "turtlebot3_burger"
	# 			target_eular = list(init_euler[:])
	# 			target_eular[2] = target_yaw
	# 			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(target_eular[0],target_eular[1],target_eular[2]))
	# 			model_state.pose.orientation = target_quat
	# 			model_state.pose.position = init_pose.position
	# 			self.pose_publisher.publish(model_state)

	# 		self.status_publisher.publish(self.free)

if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass

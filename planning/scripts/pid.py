#! /usr/bin/python

import rospy
import math
import numpy as np
import time
import tf

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class PID(object):

	def __init__(self, target_pose, mode, direction):
		self.target_pose = target_pose
		self.mode = mode
		self.direction = direction
		self.current_pose = Pose()
		self.vel = Twist()
		
		self.KP = 2.0
		self.KD = 2.0
		self.KP_rot = 25.0
		self.KD_rot = 100.0
		self.KP_rot_angular = 5.0
		self.KD_rot_angular = 100.0

		self.max_vel = 0.4
		self.max_rot = 0.3
		self.max_rot_angular = 0.3
		
		self.p_error_x = 0.0
		self.p_error_last_x = 0.0
		self.d_error_x = 0.0

		self.p_error_angular_z_linear = 0.0
		self.p_error_angular_z_linear_last = 0.0
		self.d_error_angular_z_linear  = 0.0
		
		self.p_error_angular_z_rot = 0.0
		self.p_error_angular_z_rot_last = 0.0
		self.d_error_angular_z_rot  = 0.0

		self.last_time = None

		rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.controller_status_publisher = rospy.Publisher('/Controller_Status', String, queue_size=1)

	def set_current_pose(self, current_pose):
		self.current_pose = current_pose

	def euler_from_pose(self, pose):
		quat = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		return euler

	def get_error(self, target_pose):
		error_position_x = self.target_pose.position.x - self.current_pose.position.x
		error_position_y = self.target_pose.position.y - self.current_pose.position.y

		euler_target = self.euler_from_pose(target_pose)
		euler_current = self.euler_from_pose(self.current_pose)
		error_orientation_z = euler_target[2]-euler_current[2]

		return error_position_x, error_position_y, error_orientation_z

	def get_manhattan_distance(self, goal_pose):
		return (goal_pose.position.x - self.current_pose.position.x)

	def linear_vel(self, goal_pose, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time
		
		error_dist_x,error_dist_y,_ = self.get_error(goal_pose)
		#print error_dist_x
		if self.direction == '-y':
			self.p_error_x = 0 - (error_dist_x + error_dist_y)
		else:
			self.p_error_x = error_dist_x + error_dist_y
		
		if dt == 0.0:
			return self.KP * self.max_vel * self.p_error_x

		self.d_error_x = (self.p_error_last_x - self.p_error_x) / dt
		
		self.p_error_last_x = self.p_error_x
		
		return (self.KP * self.max_vel * self.p_error_x) + (self.KD * self.max_vel * self.d_error_x)

	def get_steering_angle(self, goal_pose):
		target_euler = self.euler_from_pose(goal_pose)
		current_euler = self.euler_from_pose(self.current_pose)
		
		steer_orientation = target_euler[2] - current_euler[2]
		steer_positional_diff = math.atan2(goal_pose.position.y - self.current_pose.position.y, goal_pose.position.x - self.current_pose.position.x)
		
		if steer_positional_diff<0.2 and steer_positional_diff>-0.2:
			return steer_orientation, steer_positional_diff, current_euler
		else:
			return steer_orientation, 0.0, current_euler

	def angular_vel(self, goal_pose, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time

		error_angular_z, differntial_error, current_euler = self.get_steering_angle(goal_pose)

		if dt == 0.0:
			if self.mode=='linear':
				self.p_error_angular_z_linear = error_angular_z + differntial_error
				return (self.max_rot * self.KP_rot *(self.p_error_angular_z_linear))
			elif self.mode=='rotational':
				self.p_error_angular_z_rot = error_angular_z
				return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot)
			
		
		
		if self.mode=='linear':
			self.p_error_angular_z_linear = error_angular_z + differntial_error
			self.d_error_angular_z_linear = (self.p_error_angular_z_linear - self.p_error_angular_z_linear_last) / dt
			self.p_error_angular_z_linear_last = self.p_error_angular_z_linear

			return (self.max_rot * self.KP_rot *(self.p_error_angular_z_linear)) + (self.KD_rot * self.max_rot * (self.d_error_angular_z_linear))
		
		elif self.mode=='rotational':
			self.p_error_angular_z_rot = error_angular_z
			self.d_error_angular_z_rot = (self.p_error_angular_z_rot - self.p_error_angular_z_rot_last) / dt
			self.p_error_angular_z_rot_last = self.p_error_angular_z_rot

			return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot) + (self.KD_rot_angular * self.max_rot_angular * self.d_error_angular_z_rot)

		
			

		

	def publish_velocity(self):

		rospy.sleep(1)
		start_time = time.time()
		step_time = time.time()
		
		while step_time-start_time<10.0:
			
			if self.mode == 'linear':
				linear_velocity = self.linear_vel(self.target_pose)
				#print linear_velocity
				self.vel.linear.x = linear_velocity
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = self.angular_vel(self.target_pose)
				#self.vel.angular.z = 0.0

			elif self.mode == 'rotational':
				self.vel.linear.x = 0.0
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = self.angular_vel(self.target_pose)

			#print self.vel
			self.velocity_publisher.publish(self.vel)

			step_time = time.time()
			#print step_time-start_time

		
		self.vel.linear.x = 0.0
		self.vel.linear.y = 0.0
		self.vel.linear.z = 0.0
		self.vel.angular.x = 0.0
		self.vel.angular.y = 0.0
		self.vel.angular.z = 0.0

		self.velocity_publisher.publish(self.vel)

		self.controller_status_publisher.publish('Done')
		

	def pose_callback(self, msg):
		self.set_current_pose(msg.pose.pose)


if __name__ == "__main__":
	rospy.init_node('PID_Node')
	# pos = Pose()
	# pos.position.x = 0.5
	# pos.position.y = 0.0
	# pos.position.z = 0.0
	# pos.orientation.x = 0.0
	# pos.orientation.y = 0.0
	# pos.orientation.z = 0.7
	# pos.orientation.w = 0.7
	# pi = PID(pos, 'rotational')
	# pi.publish_velocity()
	


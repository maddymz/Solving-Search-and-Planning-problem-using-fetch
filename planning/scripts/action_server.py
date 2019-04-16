#!/usr/bin/env python

import rospy
import actionlib
import copy
from gazebo_msgs.msg import ModelState
from planning.srv import PlaceActionMsg
from planning.srv import PickActionMsg
from std_msgs.msg import String
from planning.srv import RemoveBlockedEdgeMsg
from planning.srv import MoveActionMsg
from control_msgs.msg import PointHeadAction, PointHeadGoal

from moveit_python import (MoveGroupInterface,
						   PlanningSceneInterface,
						   PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import json
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

# Point the head using controller
class PointHeadClient(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
		rospy.loginfo("Waiting for head_controller...")
		self.client.wait_for_server()

	def look_at(self, x, y, z, frame, duration=1.0):
		print("looking at ", x, y, z, frame)
		goal = PointHeadGoal()
		goal.target.header.stamp = rospy.Time.now()
		goal.target.header.frame_id = frame
		goal.target.point.x = x
		goal.target.point.y = y
		goal.target.point.z = z
		goal.min_duration = rospy.Duration(duration)
		self.client.send_goal(goal)
		self.client.wait_for_result()


# Tools for grasping
class GraspingClient(object):

	def __init__(self):
		self.scene = PlanningSceneInterface("base_link")
		self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
		self.move_group = MoveGroupInterface("arm", "base_link")

		find_topic = "basic_grasping_perception/find_objects"
		rospy.loginfo("Waiting for %s..." % find_topic)
		self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
		self.find_client.wait_for_server()

	def updateScene(self):
		# find objects
		goal = FindGraspableObjectsGoal()
		goal.plan_grasps = True
		self.find_client.send_goal(goal)
		self.find_client.wait_for_result(rospy.Duration(5.0))
		find_result = self.find_client.get_result()
		# print(find_result)
		

		# remove previous objects
		for name in self.scene.getKnownCollisionObjects():
			self.scene.removeCollisionObject(name, False)
		for name in self.scene.getKnownAttachedObjects():
			self.scene.removeAttachedObject(name, False)
		self.scene.waitForSync()

		# insert objects to scene
		idx = -1
		for obj in find_result.objects:
			idx += 1
			obj.object.name = "object%d"%idx
			self.scene.addSolidPrimitive(obj.object.name,
										 obj.object.primitives[0],
										 obj.object.primitive_poses[0],
										 wait = False)

		for obj in find_result.support_surfaces:
			# extend surface to floor, and make wider since we have narrow field of view
			height = obj.primitive_poses[0].position.z
			obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
											1.5,  # wider
											obj.primitives[0].dimensions[2] + height]
			obj.primitive_poses[0].position.z += -height/2.0

			# add to scene
			self.scene.addSolidPrimitive(obj.name,
										 obj.primitives[0],
										 obj.primitive_poses[0],
										 wait = False)

		self.scene.waitForSync()

		# store for grasping
		self.objects = find_result.objects
		self.surfaces = find_result.support_surfaces
		print("objects: ",len(self.objects))
		# print("surfaces: ",len(self.surfaces))
	
	def getGraspableCube(self):
		graspable = None
		graspable_objects = []
		for obj in self.objects:
			# print(obj.object.name)
			# print("grasps: ", len(obj.grasps))
			# need grasps
			if len(obj.grasps) < 1:
				continue
			# check size
			# if obj.object.primitives[0].dimensions[0] < 0.05 or \
			#    obj.object.primitives[0].dimensions[0] > 0.07 or \
			#    obj.object.primitives[0].dimensions[0] < 0.05 or \
			#    obj.object.primitives[0].dimensions[0] > 0.07 or \
			#    obj.object.primitives[0].dimensions[0] < 0.05 or \
			#    obj.object.primitives[0].dimensions[0] > 0.07:
			#     continue
			# # has to be on table
			# if obj.object.primitive_poses[0].position.z < 0.5:
			#     continue
			graspable_objects.append((obj.object, obj.grasps))
			# return obj.object, obj.grasps
		# nothing detected
		# return None, None
		return graspable_objects

	def getSupportSurface(self, name):
		for surface in self.support_surfaces:
			if surface.name == name:
				return surface
		return None

	def getPlaceLocation(self):
		pass

	def pick(self, block, grasps):
		# print("name: ", block.name)
		# print("grasps: ", grasps)
		# print("surface: ",block.support_surface)
		success, pick_result = self.pickplace.pick_with_retry(block.name,
															  grasps,
															  support_name=block.support_surface,
															  scene=self.scene)
		self.pick_result = pick_result
		print(self.pick_result)
		return success

	def place(self, block, pose_stamped):
		places = list()
		l = PlaceLocation()
		l.place_pose.pose = pose_stamped.pose
		l.place_pose.header.frame_id = pose_stamped.header.frame_id

		# copy the posture, approach and retreat from the grasp used
		l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
		l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
		l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
		places.append(copy.deepcopy(l))
		# create another several places, rotate each by 360/m degrees in yaw direction
		m = 16 # number of possible place poses
		pi = 3.141592653589
		for i in range(0, m-1):
			l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
			places.append(copy.deepcopy(l))

		success, place_result = self.pickplace.place_with_retry(block.name,
																places,
																scene=self.scene)
		return success

	def tuck(self):
		joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
				  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
		pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
		while not rospy.is_shutdown():
			result = self.move_group.moveToJointPosition(joints, pose, 0.02)
			if result.error_code.val == MoveItErrorCodes.SUCCESS:
				return


class RobotActionsServer:
	def __init__(self,object_dict):
		# rospy.init_node("action_server")
		self.object_dict = object_dict
		self.failure = -1
		self.success = 1
		self.empty = True
		self.status = String(data='Idle')
		self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
		self.action_publisher = rospy.Publisher("/actions",String,queue_size= 10)
		self.status_publisher = rospy.Publisher("/status",String,queue_size=10)
		self.cube = None
		rospy.Service("execute_place_action",PlaceActionMsg,self.execute_place_action)
		rospy.Service("execute_pick_action",PickActionMsg,self.execute_pick_action)
		rospy.Service("execute_move_action",MoveActionMsg,self.execute_move_action)
		self.head_action = PointHeadClient()
		self.grasping_client = GraspingClient()
		print "Action Server Initiated"

	def change_state(self,book_name,target_transform):
		model_state_msg = ModelState()
		model_state_msg.model_name = book_name
		model_state_msg.pose.position.x = target_transform[0]
		model_state_msg.pose.position.y = target_transform[1]
		model_state_msg.pose.position.z = target_transform[2]
		self.model_state_publisher.publish(model_state_msg)

	def remove_edge(self,book_name):
		rospy.wait_for_service('remove_blocked_edge')
		try:
			remove_edge = rospy.ServiceProxy('remove_blocked_edge',RemoveBlockedEdgeMsg)
			_ = remove_edge(book_name)
		except rospy.ServiceException,e:
			print "Sevice call failed: %s"%e

	def robot_at_load_location(self, state, load_locations):
		for load_location in load_locations:
			if(state[0] == load_location[0] and state[1] == load_location[1]):
				return True
		return False

	def execute_place_action(self,req):
		cube_name = req.book_name
		# bin_name = req.bin_name
		robot_state = (req.x , req.y , req.orientation)
		print("Executing Place ", cube_name)
		print("at ", robot_state[0], robot_state[1])
		print("place loc: ", self.object_dict["place_loc"])
		# print(cube_name)
		# print(self.robot_at_load_location(robot_state, self.object_dict["place_loc"]))
		# print(cube_name in self.object_dict["cubes"])
		if cube_name in self.object_dict["cubes"]:
			if self.robot_at_load_location(robot_state, self.object_dict["place_loc"]):
			# if (robot_state[0],robot_state[1]) in self.object_dict["place_loc"]:
				while not rospy.is_shutdown():
					rospy.loginfo("Placing object...")
					pose = PoseStamped()
					pose.pose = self.cube.primitive_poses[0]
					pose.pose.position.z += 0.05
					pose.header.frame_id = self.cube.header.frame_id
					if self.grasping_client.place(self.cube, pose):
						self.grasping_client.tuck()
						self.status_publisher.publish(self.status)
						return self.success
						# break
					else:
						rospy.logwarn("Placing failed.")
			else:
				print("robot not at load location")
		else:
			print("unknown cube")
			

		# if book_name in self.object_dict["books"] and bin_name in self.object_dict["bins"]:
		# 	if (robot_state[0],robot_state[1]) in self.object_dict["bins"][bin_name]["load_loc"]:
		# 		if self.object_dict["books"][book_name]["size"] == self.object_dict["bins"][bin_name]["size"]:
					# goal_loc = list(self.object_dict["bins"][bin_name]["loc"])
					# goal_loc[0] = goal_loc[0] + 0.5
					# goal_loc[1] = goal_loc[1] + 0.5
					# self.change_state(book_name, goal_loc + [3])
					# self.empty = True
				
		self.status_publisher.publish(self.status)
		return self.failure

	def execute_pick_action(self, req):
		cube_name = req.book_name
		robot_state = [req.x , req.y ,req.orientation]
		if cube_name in self.object_dict["cubes"]:
			if (robot_state[0],robot_state[1]) in self.object_dict["cubes"][cube_name]["load_loc"]:
				
				# TODO: execute pick
				# torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
				# self.head_action = PointHeadClient()
				# torso_action.move_to([1.872020, ])
				# x = self.object_dict["cubes"][cube_name]["loc"][0] # 1.000069
				# y = self.object_dict["cubes"][cube_name]["loc"][1] #1.651821
				# z = 0.029231
				# head_action.look_at(x, y, z, "map")

				pick_success = False
				while not pick_success:
					x = self.object_dict["cubes"][cube_name]["loc"][0] # 1.000069
					y = self.object_dict["cubes"][cube_name]["loc"][1] #1.651821
					z = 0.029231
					self.head_action.look_at(x, y, z, "map")
					
					# while not rospy.is_shutdown():
				
					rospy.loginfo("Searching Graspable objects...")
					self.grasping_client.updateScene()
					# cube, grasps = grasping_client.getGraspableCube()
					graspable_objects = self.grasping_client.getGraspableCube()
					if len(graspable_objects) == 0:
						rospy.logwarn("Perception failed.")
						# continue

					# Pick the block
					for obj in graspable_objects:
						if self.grasping_client.pick(obj[0], obj[1]):
							pick_success = True
							self.cube = obj[0]
						# grasping_client.pick(cube, grasps)
						else:
							rospy.logwarn("Grasping failed.")
				
				self.grasping_client.tuck()

				# quat = tf.transformations.quaternion_from_euler(r,p,s)
				# move_base.goto(x,y,quat, goto_callback)
				# rospy.Rate(3).sleep()

				# if self.empty:
				# 	self.change_state(book_name,robot_state[:2]+[2])
				# 	self.empty = False
				_ = self.remove_edge(cube_name)
				self.status_publisher.publish(self.status)
				return self.success
		self.status_publisher.publish(self.status)
		return self.failure

	def execute_move_action(self,req):
		action_str = req.actions
		print action_str
		self.action_publisher.publish(String(data=action_str))
		return self.success


if __name__ == "__main__":
	object_dict = None
	RobotActionsServer(object_dict)
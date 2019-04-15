#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from planning.srv import PlaceActionMsg
from planning.srv import PickActionMsg
from std_msgs.msg import String
from planning.srv import RemoveBlockedEdgeMsg
from planning.srv import MoveActionMsg

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
		rospy.Service("execute_place_action",PlaceActionMsg,self.execute_place_action)
		rospy.Service("execute_pick_action",PickActionMsg,self.execute_pick_action)
		rospy.Service("execute_move_action",MoveActionMsg,self.execute_move_action)
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


	def execute_place_action(self,req):
		book_name = req.book_name
		bin_name = req.bin_name
		robot_state = (req.x , req.y , req.orientation)
		if book_name in self.object_dict["books"] and bin_name in self.object_dict["bins"]:
			if (robot_state[0],robot_state[1]) in self.object_dict["bins"][bin_name]["load_loc"]:
				if self.object_dict["books"][book_name]["size"] == self.object_dict["bins"][bin_name]["size"]:
					goal_loc = list(self.object_dict["bins"][bin_name]["loc"])
					goal_loc[0] = goal_loc[0] + 0.5
					goal_loc[1] = goal_loc[1] + 0.5
					self.change_state(book_name, goal_loc + [3])
					self.empty = True
					self.status_publisher.publish(self.status)
					return self.success
		self.status_publisher.publish(self.status)
		return self.failure

	def execute_pick_action(self,req):
		book_name = req.book_name
		robot_state = [req.x , req.y ,req.orientation]
		if book_name in self.object_dict["books"]:
			if (robot_state[0],robot_state[1]) in self.object_dict["books"][book_name]["load_loc"]:
				if self.empty:
					self.change_state(book_name,robot_state[:2]+[2])
					self.empty = False
					_ = self.remove_edge(book_name)
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
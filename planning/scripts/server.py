#!/usr/bin/env python

from planning.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer 
import problem_generator
import pickle


root_path = "/home/abhyudaya/catkin_ws/src/planning/"
books = None
mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-sub', help='for providing no. of subjects', metavar='5', action='store', dest='n_subjects', default=5, type=int)
parser.add_argument('-b', help='for providing no. of books for each subject', metavar='5', action='store', dest='n_books', default=5, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
mazeScale = 0.5

def manhattanDistance(x1, y1, x2, y2):
	"""
	This function returns manhattan distance between two points.
	"""
	return abs(x1-x2) + abs(y1-y2)

def check_is_edge(edge, valueFlag):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	invalid_edges = mazeInfo[1]
	if valueFlag == "changedValuesLater":
		if edge[2] < mazeInfo[0][0] or edge[2] > mazeInfo[0][1]*mazeScale or edge[3] < mazeInfo[0][0] or edge[3] > mazeInfo[0][1]*mazeScale:
			return False
	elif valueFlag == "changedValuesBefore":
		if edge[0] < mazeInfo[0][0] or edge[0] > mazeInfo[0][1]*mazeScale or edge[1] < mazeInfo[0][0] or edge[1] > mazeInfo[0][1]*mazeScale:
			return False

	if edge in invalid_edges:
		return False
	else:
		return True

def handle_get_successor(req):
	"""
		This function returns successor of a given state. 
				
		parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
				    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
				    direction - current orientation								  direction - new orientation
				    action - current action										  g_cost - Manhatan distance from initial state to new state
															      				  hurestic_value - Manhatan distance from goal state to new state
	"""
	global mazeInfo
	directionList = ["NORTH", "EAST","SOUTH","WEST"]
	x_cord, y_cord, direction, action = req.x, req.y, req.direction, req.action

	#Checking requested action and making changes in states
	if action == 'TurnCW':
		index = directionList.index(req.direction)
		direction = directionList[(index+1)%4]
		g_cost = 2

	elif action == 'TurnCCW':
		index = directionList.index(req.direction)
		direction = directionList[(index-1)%4]
		g_cost = 2

	elif action == 'MoveF':
		if direction == "NORTH":
			y_cord += mazeScale
		elif direction == "EAST":
			x_cord += mazeScale
		elif direction == "SOUTH":
			y_cord -= mazeScale
		elif direction == "WEST":
			x_cord -= mazeScale
		g_cost = 1

	elif action == 'MoveB':
		if direction == "NORTH":
			y_cord -= mazeScale
		elif direction == "EAST":
			x_cord -= mazeScale
		elif direction == "SOUTH":
			y_cord += mazeScale
		elif direction == "WEST":
			x_cord += mazeScale
		g_cost = 3
	
	if req.x <= x_cord and req.y <= y_cord:
		isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
	else:
		isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

	if not isValidEdge:
		return GetSuccessorResponse(-1, -1, direction, -1)

	return GetSuccessorResponse(x_cord, y_cord, direction, g_cost)
  

def handle_get_initial_state(req):
	"""
	This function will return initial state of turtlebot3.
	"""
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])


def handle_is_goal_state(req):
	"""
    This function will return True if turtlebot3 is at goal state otherwise it will return False.
	"""
	global mazeInfo

	goal_state = mazeInfo[0][1]*mazeScale

	if req.x == req.y and req.x == goal_state:
		return IsGoalStateResponse(1)

	return IsGoalStateResponse(0)

def handle_get_goal_state(req):
	global mazeInfo
	goal_state = mazeInfo[0][1]*mazeScale
	return GetGoalStateResponse(goal_state,goal_state)


def remove_blocked_edge(req):
	bookname = req.bookname
	global books
	global mazeInfo
	location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
	if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
		blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
	else:
		blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
	mazeInfo[1].remove(blocked_edge)
	return "1"

def server():
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    rospy.Service('is_goal_state', IsGoalState, handle_is_goal_state)
    rospy.Service('get_goal_state',GetGoalState,handle_get_goal_state)
    rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg,remove_blocked_edge)
    print "Ready!"
    rospy.spin()
	
if __name__ == "__main__":
    args = parser.parse_args()
    n_subjects = args.n_subjects
    n_books = args.n_books
    seed = args.seed
    print n_subjects
    print n_books
    if n_subjects > 20:
    	print('Maximum no. of subjects available is: 20')
    	exit()
    book_sizes = 2
    book_count_of_each_subject = n_books * book_sizes
    book_count_list = [n_books] * n_subjects * book_sizes
    number_of_trollies = n_subjects * 2
    # grid_size = max((((book_count_of_each_subject * n_subjects) / 4) // 1 ) + 1, ((number_of_trollies/4)*7), 10)
    grid_size = 6 * n_subjects
    books, mazeInfo = generate_blocked_edges(grid_size, book_count_list, seed,  number_of_trollies, root_path,0.5)
    path = root_path + "/problem.pddl"
    problem_generator.write_pddl(path ,books)
    # pickle.dump(books,out_file)
    rospy.init_node('server')
    RobotActionsServer(books)
    # rospy.sleep(3)
   	# pprint.pprint(books)
    server()

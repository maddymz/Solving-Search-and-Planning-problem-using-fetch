#!/usr/bin/env python

import heapq
import Queue
import problem 
import rospy
from std_msgs.msg import String
import argparse
import time

rospy.init_node("algorithm")
publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)

def in_state_queue(nextstate, state_queue):
    '''
    Checks if a state exists in the state_queue
    '''
    for state in state_queue:
        if(state[0] == nextstate):
            return True
    return False

def get_manhattan_distance(from_state, to_state):
    '''
    Returns the manhattan distance between 2 states
    '''
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

def get_custom_heuristic(from_state, to_state):
    '''
    Custom heuristic function. Assumes that given we need to reach the top right corner of the grid, if the bot is facing south or west 
    then it would have to do extra work in turning to move towards the goal.
    if the orientation is west or south then extra cost is added for turns.
    returns penalty of orientation + euclidean distance 
    '''
    cost = 0
    if(from_state.orientation == "NORTH"):
        cost = 1
    elif(from_state.orientation == "EAST"):
        cost = 1
    elif(from_state.orientation == "WEST"):
        cost = 2
    elif(from_state.orientation == "SOUTH"):
        cost = 2
    return cost + ((from_state.x - to_state.x)**2 + (from_state.y - to_state.y)**2)**0.5

def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    visited = []
    state_queue = [(init_state, [])] # state queue maintains list of nodes in the queue on the order they will be visited and the actions 
                                     # needed to reach a state. Maintains tuples of (state, [list of actions to reach state])

    while(len(state_queue) > 0): # while nodes are yet to be visited

        current_state = state_queue[0][0]
        current_actions = state_queue[0][1]
        state_queue = state_queue[1:]
        visited.append(current_state)

        for action in possible_actions:
            nextstate, cost = problem.get_successor(current_state, action)
            if(nextstate.x == -1 and nextstate.y == -1): # if action not possible then do not consider
                continue
            if((nextstate not in visited) and (not in_state_queue(nextstate, state_queue))): #if state not yet visited or explored, add to queue
                if(problem.is_goal_state(nextstate)): #if goal is found then break
                    return current_actions + [action]
                else:
                    state_queue.append((nextstate, current_actions + [action]))

    return []

def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    state_queue = Queue.PriorityQueue()     # priority queue to retrieve next state based on cost of reaching it
    state_queue.put((0, (init_state, [])))  # maintains states in tuples as (cost, (state, [list of actions to reach the state]))
    visited = []
    state_cost = {}

    while(not state_queue.empty()):
        top_item = state_queue.get()
        current_cost = top_item[0]
        current_state = top_item[1][0]
        current_actions = top_item[1][1]
        
        if(current_state in visited): # since priority queues do not provide functionality for editing cost of existing elements. 
            continue                  # We can discard old stale states that are already marked visited.
        
        if(problem.is_goal_state(current_state)):
            break

        visited.append(current_state)
        for action in possible_actions:
            nextstate, cost = problem.get_successor(current_state, action)
            key = (nextstate.x, nextstate.y, nextstate.orientation) # tuple of state is created to be used in the dictionary
            if(nextstate.x == -1 and nextstate.y == -1):
                continue
            #if a new state is found or the state has lesser cost than it's existing value then we update the cost and add the state to the queue
            if((nextstate not in visited and key not in state_cost.keys()) or (key in state_cost.keys() and current_cost + cost < state_cost[key])):
                state_queue.put((current_cost + cost, (nextstate, current_actions + [action])))
                state_cost[key] = current_cost + cost

    if(problem.is_goal_state(current_state)):
        action_list = current_actions

    return action_list

def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    state_queue = Queue.PriorityQueue()
    state_queue.put((get_manhattan_distance(init_state, goal_state), (init_state, [])))
    visited = []
    state_cost = {}

    while(not state_queue.empty()):
        top_item = state_queue.get()
        current_cost = top_item[0]
        current_state = top_item[1][0]
        current_actions = top_item[1][1]
        
        if(current_state in visited):
            continue
        
        if(problem.is_goal_state(current_state)):
            break

        visited.append(current_state)
        for action in possible_actions:
            nextstate, cost = problem.get_successor(current_state, action)
            # cost = get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
            cost = get_custom_heuristic(nextstate, goal_state) # custom heuristic for q8.2
            key = (nextstate.x, nextstate.y, nextstate.orientation)
            if(nextstate.x == -1 and nextstate.y == -1):
                continue
            # if a new state is found then add to queue
            if(nextstate not in visited and key not in state_cost.keys()):
                state_queue.put((cost, (nextstate, current_actions + [action])))
                state_cost[key] = cost
            

    if(problem.is_goal_state(current_state)):
        action_list = current_actions

    return action_list

def astar():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    state_queue = Queue.PriorityQueue()
    state_queue.put((0 + get_manhattan_distance(init_state, goal_state), (0, init_state, [])))
    visited = []
    state_cost = {}

    while(not state_queue.empty()):
        top_item = state_queue.get()
        current_cost = top_item[1][0]
        current_state = top_item[1][1]
        current_actions = top_item[1][2]
        
        if(current_state in visited):
            continue
        
        if(problem.is_goal_state(current_state)):
            break

        visited.append(current_state)
        for action in possible_actions:
            nextstate, cost = problem.get_successor(current_state, action)
            # heuristic = get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
            heuristic = get_custom_heuristic(nextstate, goal_state) # custom heuristic for q8.2
            key = (nextstate.x, nextstate.y, nextstate.orientation)
            if(nextstate.x == -1 and nextstate.y == -1):
                continue
            # if a new state is found or an explored state with lower cost value is found then add to queue
            if((nextstate not in visited and key not in state_cost.keys()) or (key in state_cost.keys() and current_cost + cost + heuristic < state_cost[key])):
                state_queue.put((current_cost + cost + heuristic, (current_cost + cost, nextstate, current_actions + [action])))
                state_cost[key] = current_cost + cost + heuristic

    if(problem.is_goal_state(current_state)):
        action_list = current_actions

    return action_list
   

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print("Incorrect Algorithm name.")
        exit(1)
    start_time = time.time()
    actions = algorithm()
    print("Execution time : ", time.time() - start_time )
    print(actions)
    exec_action_list(actions)
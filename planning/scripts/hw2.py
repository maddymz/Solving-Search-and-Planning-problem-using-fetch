#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from planning.srv import MoveActionMsg
from planning.srv import PlaceActionMsg
from planning.srv import PickActionMsg
import problem
import json
import Queue
from subprocess import Popen, PIPE
import timeit

class Executioner:

    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.json_file_path = r"/home/abhyudaya/catkin_ws/src/planning/books.json" #path to books.json
        self.plan_path = r"sas_plan" #path to generated plan file
        self.domain_file_path = r'/home/abhyudaya/catkin_ws/src/Planning/domain.pddl' #path to domain file
        self.problem_file_path = r'/home/abhyudaya/catkin_ws/src/Planning/problem.pddl' #path to problem file
        self.FD_file_path = r'/home/abhyudaya/catkin_ws/src/Planning/scripts/FD/fast-downward.py' #path to FD folder
        with open(self.json_file_path) as f:
            self.env_data = json.load(f)

        start = timeit.default_timer()
        self.actions = self.parse_plan()
        end = timeit.default_timer()
        print("Plan found in: ", end - start)
        print("******************REFINED ACTION PLAN**************************************")
        for action in self.actions:
            print(action)
        print("***************************************************************************")
        
        self.action_index = 0
        self.current_state = problem.get_initial_state()
        self.status_subscriber = rospy.Subscriber('/status', String, self.status_callback)
        self.execute_action()
        rospy.spin()
    
    def status_callback(self, data):
        if(data.data == "Idle"):
            self.execute_action()

    def execute_action(self):
        if(self.action_index >= len(self.actions)):
            return
        action = self.actions[self.action_index]
        self.action_index += 1
        print("Executing ", action)
        if(action[0] == "move"):
            move_seq = action[1]
            goal_location = action[2]
            if(move_seq == ""): #if already at destination
                return
            problem.execute_move_action(move_seq)
            self.current_state = goal_location
            print("Reached location:", self.current_state.x, self.current_state.y, self.current_state.orientation)
        elif(action[0] == "pick"):
            book_name = action[1]
            response = problem.execute_pick_action(book_name, self.current_state)
            if(response == -1):
                print("Unsuccessful Pick")
        elif(action[0] == "place"):
            book_name = action[1]
            bin_name = action[2]
            response = problem.execute_place_action(book_name, bin_name, self.current_state)
            if(response == -1):
                print("Unsuccessful Place")

    def in_state_queue(self, nextstate, state_queue):
        '''
        Checks if a state exists in the state_queue
        '''
        for state in state_queue:
            if(state[0] == nextstate):
                return True
        return False

    def is_goal_state(self, current_state, goal_state):
        if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False
    
    def get_manhattan_distance(self, from_state, to_state):
        '''
        Returns the manhattan distance between 2 states
        '''
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

    def build_goal_states(self, locations):
        states = []
        for location in locations:
            states.append(problem.State(location[0], location[1], "EAST"))
            states.append(problem.State(location[0], location[1], "WEST"))
            states.append(problem.State(location[0], location[1], "NORTH"))
            states.append(problem.State(location[0], location[1], "SOUTH"))
        return states

    def get_path_gbfs(self, init_state, goal_locations):
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False
        for goal_state in goal_states: #search for any of the load locations
            possible_actions = problem.get_actions()
            action_list = []

            state_queue = Queue.PriorityQueue()
            state_queue.put((self.get_manhattan_distance(init_state, goal_state), (init_state, [])))
            visited = []
            state_cost = {}

            while(not state_queue.empty()):
                top_item = state_queue.get()
                current_cost = top_item[0]
                current_state = top_item[1][0]
                current_actions = top_item[1][1]
                
                if(current_state in visited):
                    continue
                
                if(self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break

                visited.append(current_state)
                for action in possible_actions:
                    nextstate, cost = problem.get_successor(current_state, action)
                    cost = self.get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
                    key = (nextstate.x, nextstate.y, nextstate.orientation)
                    if(nextstate.x == -1 and nextstate.y == -1):
                        continue
                    # if a new state is found then add to queue
                    if(nextstate not in visited and key not in state_cost.keys()):
                        state_queue.put((cost, (nextstate, current_actions + [action])))
                        state_cost[key] = cost
                    
            if(self.is_goal_state(current_state, goal_state)):
                action_list = current_actions
                final_state = current_state
                goal_reached = True
                break

        return action_list, final_state, goal_reached

    def get_load_locations(self, location):
        obj = location[:location.index("_iloc")]
        if(obj.startswith("book")):
            return self.env_data["books"][obj]["load_loc"]
        elif(obj.startswith("trolly")):
            return self.env_data["bins"][obj]["load_loc"]

    def parse_plan(self):
        # run_planner_command = self.FD_file_path + " " + self.domain_file_path + " " + self.problem_file_path + " --search \"lazy_greedy([ff()], preferred=[ff()])\""
        # process = Popen(run_planner_command, stdout=PIPE, stderr=PIPE, shell=True)
        # stdout, stderr = process.communicate()
        # # print(stdout) #print output of planner
        # # print(stderr)
        # process.wait()
        current_state = problem.get_initial_state()
        actions = []
        # f = open(self.plan_path, mode='r')
        f = ["move fetch x book_1_iloc"]
        for line in f:
            # print(line)
            line = line.strip() #remove whitespace
            # line = line[1:] #remove brackets
            # line = line[:-1]
            args = line.split(' ')
            # print(args)
            if(args[0] == "move"): #perform downward refinement
                from_location = args[2]
                to_location = args[3]
                load_locations = self.get_load_locations(to_location)
                print(from_location, to_location)
                print(current_state.x, current_state.y, " to ", load_locations)
                move_seq, current_state, goal_reached = self.get_path_gbfs(current_state, load_locations)
                if(not goal_reached):
                    print("Path not possible. Rerun")
                    return
                actions.append(("move", move_seq, current_state))
            elif(args[0] == "pick"):
                book = args[1]
                actions.append(("pick", book))
            elif(args[0] == "place"):
                book = args[1]
                bin_ = args[4]
                actions.append(("place", book, bin_))
        return actions

if __name__ == "__main__":
    try:
        Executioner()
    except rospy.ROSInterruptException:
        pass
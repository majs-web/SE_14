
import random
from collections import deque
from  planning.planning_problem import Planning_Problem 

class Planner():
    """
        Planner class with different planning method implementations
    """

    @classmethod
    def random_search(cls, start_state):
        """
            Implementation of random search.

            returns (path, step_count) where path is a list of actions to the goal and step_count is the number of states extended
        """

        open_list = [(start_state,[])] # this is a list of state,path. The path of the start state is empty.
        start_state["path"]  = [] # small hack: we add paths to the state in order to remember the path we travelled.
        step_count = 0
        
        while len(open_list) > 0:
            state, path = random.choice(open_list)
            step_count+= 1

            for action in Planning_Problem.actions:
                next_state = Planning_Problem.state_transition(state,action)
                next_path = path + [action]

                if Planning_Problem.goal(next_state):
                    return next_path, step_count
                else:
                    open_list.append((next_state,next_path))

        return None, step_count
    

    @classmethod
    def breadth_first_search(cls, start_state):
        """
            Implementation of Breadth First Search

            returns (path, step_count) where path is a list of actions to the goal and step_count is the number of states extended
        """
        
        open_list = [(start_state,[])] # this is a list of state,path. The path of the start state is empty.
        start_state["path"]  = [] # small hack: we add paths to the state in order to remember the path we travelled.
        visited = [start_state] 
        step_count = 0

        while open_list:
            state, path = open_list.pop(0) # takes the earliest queued state
            step_count += 1

            # returns the path until now, if the state fits with the goal 
            if Planning_Problem.goal(state):
                return path, step_count
            
            # computes what the next state will be for each possible/available action 
            for action in Planning_Problem.actions:
                next_state = Planning_Problem.state_transition(state, action)

                # checks the visited list - if its already there, it skips it, to avoid repeats/infinite loops
                if next_state in visited:
                    continue

                # creates a new list of the current path + the new path
                next_path = path + [action]

                # if the upcoming path is same as the goal, it returns with the path
                if Planning_Problem.goal(next_state):
                    return next_path, step_count
                
                # if the goal was not yet reached, appends to visited
                visited.append(next_state)
                open_list.append((next_state, next_path))

        return None, step_count


    @classmethod
    def shortest_toast_time_search(cls, start_state):
        """
        Implementation of a search algorithm that also optimized for the time it takes to toast a slice of bread.
        This time is measured within the state in attribute ["time"]

        returns (path, step_count) where path is a list of actions to the goal and step_count is the number of states extended
        """
        # uses Dijkstra's algorithm concept

        step_count = 0

        # ignores "time"
        def key(s):
            return (s["toaster_has_power"], s["toaster_is_on"], s["bread_location"], s["bread_state"])
        
        # starts with one state to explore, contains tuples of time, state, and path
        open_list = [(start_state["time"], start_state, [])]

        # stores the lowest time found - if there comes a better time later, it exchanges time/route
        best_time = { key(start_state): start_state["time"] }

        while open_list:
            # assumes the first entry is the one with the lowest time
            best_t = 0
            
            # goes through the list to find something with lower time and exchange it
            for t in range(1, len(open_list)):
                if open_list[t][0] < open_list[best_t] [0]:
                    best_t = t
                
            # checks the current best partial path
            this_time, state, path = open_list.pop(best_t)
            step_count += 1

            # to avoid adding steps when popping old/outdated times
            k = key(state)
            if this_time != best_time.get(k):
                continue

            # finishes if the route that gets popped is the one with the lowest time
            if Planning_Problem.goal(state):
                return path, step_count
            
            # tries all possible actions to see which is has a lower time
            for action in Planning_Problem.actions:
                next_state = Planning_Problem.state_transition(state, action)
                nk = key(next_state)
                next_time = next_state["time"]

                # checks if the point reached has been reached before with lower time
                # -> if this point has not been reached before or if it has but this time with lower time
                # if neither is true, it's a worse route than before
                if (nk not in best_time) or (next_time < best_time[nk]):
                    best_time[nk] = next_time
                    open_list.append((next_time, next_state, path + [action]))

        return None, step_count

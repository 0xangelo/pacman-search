# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    visited = set()
    border = []
    start = problem.getStartState()
    
    def dfs_recur(state, visited, border, stack):

        if state not in visited:
            if problem.isGoalState(state):
                stack.push(border)
                return
            
            visited.add(state)
            for (next_state, action, stepCost) in problem.getSuccessors(state):
                dfs_recur(next_state, visited, border + [action], stack)
                if not stack.isEmpty():
                    return
        else:
            return

    dfs_recur(start, visited, border, stack)
    return stack.pop()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # start = problem.getStartState()
    # visited = set()
    # queue = util.Queue()
    # came_from = {}

    # def path(state, came_from):
    #     directions = []
    #     start = problem.getStartState()

    #     while state != start:
    #         (state, action) = came_from[state]
    #         directions.insert(0, action)
            
    #     return directions

    # visited.add(start)
    # queue.push(start)

    # while not queue.isEmpty():
    #     state = queue.pop()
        
    #     if problem.isGoalState(state):
    #         return path(state, came_from)

    #     for (next_state, action, stepCost) in problem.getSuccessors(state):
    #         if next_state not in visited:
    #             visited.add(next_state)
    #             came_from[next_state] = (state, action)
    #             queue.push(next_state)
    
    
    start = problem.getStartState()
    visited = []
    queue = util.Queue()

    visited.append(start)
    queue.push( (start, []) )

    while not queue.isEmpty():
        (state, path) = queue.pop()
        
        if problem.isGoalState(state):
            return path

        for (next_state, action, stepCost) in problem.getSuccessors(state):
            if next_state not in visited:
                visited.append(next_state)
                queue.push( (next_state, path + [action]) )
    
    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    visited = set()
    priority_queue = util.PriorityQueue()
    came_from = {}
    g_cost= {}

    def path(state, came_from):
        directions = []
        start = problem.getStartState()

        while state != start:
            (state, action) = came_from[state]
            directions.insert(0, action)
            
        return directions

    visited.add(start)
    g_cost[start] = 0
    priority_queue.push(start, g_cost[start])

    while not priority_queue.isEmpty():
        state = priority_queue.pop()
        if problem.isGoalState(state):
            return path(state, came_from)
        
        for (next_state, action, stepCost) in problem.getSuccessors(state):
            if next_state not in visited:
                visited.add(next_state)
                g_cost[next_state] = g_cost[state] + stepCost
                came_from[next_state] = (state, action)
                priority_queue.push(next_state, g_cost[next_state])
                
            elif g_cost[next_state] > g_cost[state] + stepCost:
                g_cost[next_state] = g_cost[state] + stepCost
                came_from[next_state] = (state, action)
                priority_queue.update(next_state, g_cost[next_state])
                


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # start = problem.getStartState()
    # visited = set()
    # priority_queue = util.PriorityQueue()
    # came_from = {}
    # g_cost = {}

    # def path(state, came_from):
    #     directions = []
    #     start = problem.getStartState()

    #     while state != start:
    #         (state, action) = came_from[state]
    #         directions.insert(0, action)
            
    #     return directions

    # visited.add(start)
    # g_cost[start] = 0
    # priority_queue.push(start, heuristic(start, problem))

    # while not priority_queue.isEmpty():
    #     state = priority_queue.pop()
                        
    #     if problem.isGoalState(state):
    #         return path(state, came_from)

    #     for (next_state, action, stepCost) in problem.getSuccessors(state):
    #         if next_state not in visited:
    #             visited.add(next_state)
    #             came_from[next_state] = (state, action)
    #             g_cost[next_state] = g_cost[state] + stepCost
    #             priority_queue.push(next_state, heuristic(next_state, problem) + g_cost[next_state])
                
    #         elif g_cost[next_state] > g_cost[state] + stepCost:
    #             came_from[next_state] = (state, action)
    #             g_cost[next_state] = g_cost[state] + stepCost
    #             priority_queue.update(next_state, heuristic(next_state, problem) + g_cost[next_state])

    start = problem.getStartState()
    visited = list()
    priority_queue = util.PriorityQueue()
    g_cost = {}

    visited.append(start)
    g_cost[start[0]] = 0
    priority_queue.push((start, []), heuristic(start, problem))

    while not priority_queue.isEmpty():
        (state, path) = priority_queue.pop()
                        
        if problem.isGoalState(state):
            return path

        for (next_state, action, stepCost) in problem.getSuccessors(state):
            new_path = path + [action]
            cost = problem.getCostOfActions(new_path) + heuristic(next_state, problem)

            if next_state not in visited:
                visited.append(next_state)
                g_cost[next_state[0]] = cost
                priority_queue.push((next_state, new_path), cost)
                
            elif next_state != start and g_cost[next_state[0]] > cost:
                g_cost[next_state[0]] = cost
                priority_queue.update((next_state, new_path), cost)
                
                

def learningRealTimeAStar(problem, heuristic=nullHeuristic):
    """Execute a number of trials of LRTA* and return the best plan found."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

    # MAXTRIALS = ...
    

# Abbreviations 
# *** DO NOT CHANGE THESE ***
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
lrta = learningRealTimeAStar

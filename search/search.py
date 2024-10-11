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
from lib2to3.pytree import Node

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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    # "visited" contains nodes which have been popped from the stack,
    # and the direction from which they were obtained
    visited = {} # Dictionary
    # "solution" contains the sequence of directions for Pacman to get to the goal state
    solution = [] #
    # "stack" contains triplets of: (node in the fringe list, direction, cost)
    stack = util.Stack()
    # "parents" contains nodes and their parents
    parents = {}

    # start state is obtained and added to the stack
    start = problem.getStartState()
    stack.push((start, 'Undefined', 0))
    # the direction from which we arrived in the start state is undefined
    visited[start] = 'Undefined'

    # return if start state itself is the goal
    if problem.isGoalState(start):
        return solution

    # loop while stack is not empty and goal is not reached
    goal = False;
    while (stack.isEmpty() != True and goal != True):
        # pop from top of stack
        node = stack.pop()
        # store element and its direction
        visited[node[0]] = node[1]
        # check if element is goal
        if problem.isGoalState(node[0]):
            node_sol = node[0]
            goal = True
            break
        # expand node
        for elem in problem.getSuccessors(node[0]):
            print(problem.getSuccessors(node[0]), "something")
            # if successor has not already been visited
            if elem[0] not in visited.keys():
                # store successor and its parent
                parents[elem[0]] = node[0]
                # push successor onto stack
                stack.push(elem)

    # finding and storing the path
    while (node_sol in parents.keys()):
        # find parent
        node_sol_prev = parents[node_sol]
        # prepend direction to solution
        solution.insert(0, visited[node_sol])
        # go to previous node
        node_sol = node_sol_prev

    return solution

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    '''This function pushes non-visited nodes onto the queue.
    Nodes are popped one by one, and the following steps are performed:
    1. The node is marked as visited.
    2. If it is a goal node, the loop stops, and the solution is obtained by backtracking using stored parents.
    3. If it is not a goal node, it is expanded.
    4. If the successor node is not visited, and has not been expanded as a child of another node,
       then it is pushed onto the queue and its parent is stored.'''

    # "visited" contains nodes which have been popped from the queue,
    # and the direction from which they were obtained
    visited = {} # Dictionary dict = { "brand": "Ford" }
    # "solution" contains the sequence of directions for Pacman to get to the goal state
    solution = [] # List
    # "queue" contains triplets of: (node in the fringe list, direction, cost)
    queue = util.Queue()
    # "parents" contains nodes and their parents
    parents = {}

    # start state is obtained and added to the queue
    start = problem.getStartState()
    queue.push((start, 'Undefined', 0))
    # the direction from which we arrived in the start state is undefined
    visited[start] = 'Undefined'

    # return if start state itself is the goal
    if problem.isGoalState(start):
        return solution

    # loop while queue is not empty and goal is not reached
    goal = False
    while (queue.isEmpty() != True and goal != True):
        # pop from top of queue
        node = queue.pop()
        # store element and its direction
        visited[node[0]] = node[1] # In a dictionary, add the node you're at and how you got there (direction)
        # check if element is goal
        if problem.isGoalState(node[0]):
            node_sol = node[0]
            goal = True
            break
        # expand node
        for elem in problem.getSuccessors(node[0]):
            # if successor has not already been visited or expanded as a child of another node
            if elem[0] not in visited.keys() and elem[0] not in parents.keys():
                # store successor and its parent
                parents[elem[0]] = node[0]
                # push successor onto queue
                queue.push(elem)

    # finding and storing the path
    while (node_sol in parents.keys()):
        # find parent
        node_sol_prev = parents[node_sol]
        # prepend direction to solution
        solution.insert(0, visited[node_sol])
        # go to previous node
        node_sol = node_sol_prev

    return solution

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

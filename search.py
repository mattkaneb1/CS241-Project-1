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

class Node:
	parent = None
	state = []
        pathCost = 0

	def __init__(self,parent):
		self.parent = parent

	def setState(self,state):
		self.state = state

        def setPathCost(self, parentPC):
                self.pathCost = parentPC + self.state[2]
                
	def getState(self):
                return self.state

	def getParent(self):
		return self.parent	

	def getCoordinates(self):
		return self.state[0]

	def getAction(self):
		return self.state[1]

	def getCost(self):
		return self.state[2]

        def getPathCost(self):
                return self.pathCost
        
                
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
	"""
    #print "Start:", problem.getStartState()
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())

    stack = util.Stack()
    start = Node(None)
    start.setState((problem.getStartState(),None,0))
    stack.push(start)

    explored = util.Counter()

    while(True):
    	if stack.isEmpty():
    		return "ERROR"
    	else:
    		n = stack.pop()
    		if (problem.isGoalState(n.getCoordinates())):
    			return getNodePath(n)

                if n.getCoordinates() not in explored:

                        explored[n.getCoordinates()] = True
    		        successors = problem.getSuccessors(n.getCoordinates())
                
    		        for s in successors:
    			        node = Node(n)
    			        node.setState(s)
    				stack.push(node)


def getNodePath(node):
	path = []

	while(True):
		#print(node.getState())
		#print(node.getAction())
		if node.getAction() == None:
			path.reverse()
			return path
		path.append(node.getAction())
		node = node.parent

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    q = util.Queue()
    start = Node(None)
    start.setState((problem.getStartState(),None,0))
    q.push(start)

    explored = util.Counter()
    
    while(True):
    	if q.isEmpty():
    		return "ERROR"
    	else:
    		n = q.pop()
    		if (problem.isGoalState(n.getCoordinates())):
    			return getNodePath(n)

                if n.getCoordinates() not in explored:

                        explored[n.getCoordinates()] = True
    		        successors = problem.getSuccessors(n.getCoordinates())
                
    		        for s in successors:
    			        node = Node(n)
    			        node.setState(s)
    				q.push(node)

                                
def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    q = util.PriorityQueue()
    start = Node(None)
    start.setState((problem.getStartState(),None,0))
    q.push(start, 0)

    explored = util.Counter()
    
    while(True):
    	if q.isEmpty():
    		return "ERROR"
    	else:
    		n = q.pop()                
    		if (problem.isGoalState(n.getCoordinates())):
    			return getNodePath(n)

                if n.getCoordinates() not in explored:

                        explored[n.getCoordinates()] = True 
    		        successors = problem.getSuccessors(n.getCoordinates())

    		        for s in successors:
    			        node = Node(n)
    			        node.setState(s)
                                node.setPathCost(n.getPathCost())
                               	q.push(node, node.getPathCost())
                                

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    q = util.PriorityQueue()
    start = Node(None)
    start.setState((problem.getStartState(),None,0))
    q.push(start, heuristic(start.getCoordinates(), problem))

    explored = util.Counter()

    while(True):
    	if q.isEmpty():
    		return "ERROR"
    	else:
    		n = q.pop()
    		if (problem.isGoalState(n.getCoordinates())):
                        return getNodePath(n)

                if n.getCoordinates() not in explored:
                        explored[n.getCoordinates()] = True 
    		        successors = problem.getSuccessors(n.getCoordinates())
                
    		        for s in successors:
    			        node = Node(n)
    			        node.setState(s)
                                node.setPathCost(n.getPathCost())
    				q.push(node, node.getPathCost()+heuristic(node.getCoordinates(),problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

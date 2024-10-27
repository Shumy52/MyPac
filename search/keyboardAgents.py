# keyboardAgents.py
# -----------------
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



import random

from game import Directions, Agent, Actions
from graphicsDisplay import PacmanGraphics


class KeyboardAgent(Agent):
    """
    An agent controlled by the keyboard.
    """
    # NOTE: Arrow keys also work.
    WEST_KEY  = 'a'
    EAST_KEY  = 'd'
    NORTH_KEY = 'w'
    SOUTH_KEY = 's'
    STOP_KEY = 'q'

    def __init__( self, index = 0 ):

        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []

    def getAction( self, state):
        from graphicsUtils import keys_waiting
        from graphicsUtils import keys_pressed
        keys = list(keys_waiting()) + list(keys_pressed())
        if keys != []:
            self.keys = keys

        legal = state.getLegalActions(self.index)
        move = self.getMove(legal)

        if move == Directions.STOP:
            # Try to move in the same direction as before
            if self.lastMove in legal:
                move = self.lastMove

        if (self.STOP_KEY in self.keys) and Directions.STOP in legal: move = Directions.STOP

        if move not in legal:
            move = random.choice(legal)

        self.lastMove = move
        return move

    def getMove(self, legal):
        move = Directions.STOP
        if   (self.WEST_KEY in self.keys or 'Left' in self.keys) and Directions.WEST in legal:  move = Directions.WEST
        if   (self.EAST_KEY in self.keys or 'Right' in self.keys) and Directions.EAST in legal: move = Directions.EAST
        if   (self.NORTH_KEY in self.keys or 'Up' in self.keys) and Directions.NORTH in legal:   move = Directions.NORTH
        if   (self.SOUTH_KEY in self.keys or 'Down' in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
        return move

# class KeyboardAgent2(KeyboardAgent):
#     """
#     A second agent controlled by the keyboard.
#     """
#     # NOTE: Arrow keys also work.
#     WEST_KEY  = 'j'
#     EAST_KEY  = "l"
#     NORTH_KEY = 'i'
#     SOUTH_KEY = 'k'
#     STOP_KEY = 'u'
#
#     def getMove(self, legal):
#         move = Directions.STOP
#         if   (self.WEST_KEY in self.keys) and Directions.WEST in legal:  move = Directions.WEST
#         if   (self.EAST_KEY in self.keys) and Directions.EAST in legal: move = Directions.EAST
#         if   (self.NORTH_KEY in self.keys) and Directions.NORTH in legal:   move = Directions.NORTH
#         if   (self.SOUTH_KEY in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
#         return move

class KeyboardAgent2(KeyboardAgent):
    """
    A second agent controlled by the keyboard, with a corruption mechanic.
    When the corruption meter hits 100%, Pacman becomes possessed and seeks the nearest ghost.
    """
    # Define keys for movement
    WEST_KEY = 'a'
    EAST_KEY = "d"
    NORTH_KEY = 'w'
    SOUTH_KEY = 's'
    STOP_KEY = 'q'

    def __init__(self):
        super().__init__()
        self.corruption = 30  # Start with 30% corruption
        self.corruption_rate = 100  # Percent per second increase
        self.corruption_degradation = 5 # Percent to drop the corruption (in Christ's own unit of measurement
        self.possessed = False  # Possession flag
        self.time_possessed = 0  # Time left for possession
        self.possession_time=0

    # def updateCorruption(self, delta_time):
    #     """
    #     Increases the corruption meter over time. If it hits 100%, possession begins.
    #     """
    #     if not self.possessed:
    #         self.corruption += self.corruption_rate * delta_time
    #         if self.corruption >= 100:
    #             self.startPossession()`

    def update(self, delta_time, state):
        """
        Update the agent's state, including corruption mechanics and storing game state.
        """
        # Store the current state for use in moveTowardsGhost
        self.current_state = state

        # print(self.corruption)

        # Update corruption meter and possession status
        self.corruption += delta_time * self.corruption_rate

        if self.possessed:
            self.possession_time -= delta_time * 100 * self.corruption_degradation
            # print(self.possession_time)
            PacmanGraphics.setCorrupted(PacmanGraphics, True, self.possession_time)
            if self.possession_time <= 0:
                self.possessed = False
                self.corruption = 0
                PacmanGraphics.setCorrupted(PacmanGraphics, False, self.corruption)
        elif self.corruption >= 100:
            PacmanGraphics.setCorrupted(PacmanGraphics, True, self.possession_time)
            self.possessed = True
            self.possession_time = 100  # Pacman is possessed for 3-6 seconds
            # print(self.possessed)
        else:
            PacmanGraphics.setCorrupted(PacmanGraphics, False, self.corruption)

    # TODO: Be able to select between the 2 modes of moving from cmd param
    def getMove(self, legal):
        """
        Chooses the move for Pacman. If possessed, ignores keyboard input and seeks ghosts.
        Otherwise, listens to keyboard input.
        """
        # Check if Pacman is possessed
        if self.possessed:
            return self.moveTowardsGhost(self.current_state) # REPAIRED
        else:
            # If not possessed, return normal keyboard movement
            move = Directions.STOP
            if (self.WEST_KEY in self.keys) and Directions.WEST in legal:  move = Directions.WEST
            if (self.EAST_KEY in self.keys) and Directions.EAST in legal: move = Directions.EAST
            if (self.NORTH_KEY in self.keys) and Directions.NORTH in legal: move = Directions.NORTH
            if (self.SOUTH_KEY in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
            return move

    # def getMove(self, legal):
    #     """
    #     Chooses the move for Pacman. If possessed, ignores keyboard input and seeks ghosts.
    #     Otherwise, listens to keyboard input.
    #     """
    #     if self.possessed:
    #         # Use Alpha-Beta Pruning to choose move when possessed
    #         _, action = self.alphabeta(self.current_state, 0, 0, float('-inf'), float('inf'), maxDepth=3)
    #         return action
    #     else:
    #         move = Directions.STOP
    #         if (self.WEST_KEY in self.keys) and Directions.WEST in legal:  move = Directions.WEST
    #         if (self.EAST_KEY in self.keys) and Directions.EAST in legal: move = Directions.EAST
    #         if (self.NORTH_KEY in self.keys) and Directions.NORTH in legal: move = Directions.NORTH
    #         if (self.SOUTH_KEY in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
    #         return move

    def moveTowardsGhost(self, state):
        """
        Greedy method for moving towards the nearest ghost.
        Chooses a legal move that gets Pacman closer to the nearest ghost.
        """
        ghost_positions = state.getGhostPositions()
        pacman_position = state.getPacmanPosition()

        # Find the nearest ghost
        nearest_ghost = min(ghost_positions, key=lambda pos: self.getDistance(pacman_position, pos))

        # Get the legal move that brings Pacman closer to the nearest ghost
        best_move = Directions.STOP
        best_distance = float('inf')
        legal = state.getLegalPacmanActions()
        for action in legal:
            successor_pos = self.getSuccessorPosition(pacman_position, action)
            distance = self.getDistance(successor_pos, nearest_ghost)
            if distance < best_distance:
                best_move = action
                best_distance = distance

        return best_move

    def getDistance(self, pos1, pos2):
        """
        Calculate Manhattan distance between two points.
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def getSuccessorPosition(self, position, action):
        """
        Get the position after taking the given action.
        """
        x, y = position
        dx, dy = Actions.directionToVector(action)
        return (int(x + dx), int(y + dy))

    # For alphabeta/minmax

    def evaluateState(self, state): #In the class
        """
        Evaluation function to score a state based on Pacman's distance to the nearest ghost.
        """
        pacman_position = state.getPacmanPosition()
        ghost_positions = state.getGhostPositions()

        # Calculate the distance to the nearest ghost
        closest_ghost_distance = min([self.getDistance(pacman_position, ghost) for ghost in ghost_positions])

        # Return a higher score for closer distances (since Pacman wants to reach the ghosts)
        return -closest_ghost_distance  # Negative since closer distance is more "desirable" when possessed

    # TODO: this code sucks
    def alphabeta(self, state, depth, agentIndex, alpha, beta, maxDepth):
        if depth == maxDepth or state.isWin() or state.isLose():
            return self.evaluateState(state), None

        if agentIndex == 0:  # Pacman's move
            best_score = float('-inf')
            best_action = None
            for action in state.getLegalPacmanActions():
                successor = state.generateSuccessor(agentIndex, action)
                score, _ = self.alphabeta(successor, depth, 1, alpha, beta, maxDepth)
                if score > best_score:
                    best_score, best_action = score, action
                alpha = max(alpha, best_score)
                if beta <= alpha:
                    break
            return best_score, best_action

        else:  # Ghost's move
            best_score = float('inf')
            best_action = None
            for action in state.getLegalGhostActions(agentIndex):
                successor = state.generateSuccessor(agentIndex, action)
                next_agent = 0 if agentIndex == state.getNumAgents() - 1 else agentIndex + 1
                new_depth = depth + 1 if next_agent == 0 else depth
                score, _ = self.alphabeta(successor, new_depth, next_agent, alpha, beta, maxDepth)
                if score < best_score:
                    best_score, best_action = score, action
                beta = min(beta, best_score)
                if beta <= alpha:
                    break
            return best_score, best_action


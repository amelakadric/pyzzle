from collections import deque
import heapq
import queue
import random
import time
from queue import PriorityQueue, Queue

import config
from state import is_solvable


class Algorithm:
    def __init__(self, heuristic=None):
        self.heuristic = heuristic
        self.nodes_evaluated = 0
        self.nodes_generated = 0

    def get_legal_actions(self, state):
        self.nodes_evaluated += 1
        max_index = len(state)
        zero_tile_ind = state.index(0)
        legal_actions = []
        if 0 <= (up_ind := (zero_tile_ind - config.N)) < max_index:
            legal_actions.append(up_ind)
        if 0 <= (right_ind := (zero_tile_ind + 1)) < max_index and right_ind % config.N:
            legal_actions.append(right_ind)
        if 0 <= (down_ind := (zero_tile_ind + config.N)) < max_index:
            legal_actions.append(down_ind)
        if (
            0 <= (left_ind := (zero_tile_ind - 1)) < max_index
            and (left_ind + 1) % config.N
        ):
            legal_actions.append(left_ind)
        return legal_actions

    def apply_action(self, state, action):
        self.nodes_generated += 1
        copy_state = list(state)
        zero_tile_ind = state.index(0)
        copy_state[action], copy_state[zero_tile_ind] = (
            copy_state[zero_tile_ind],
            copy_state[action],
        )
        return tuple(copy_state)

    def get_steps(self, initial_state, goal_state):
        pass

    def get_solution_steps(self, initial_state, goal_state):
        begin_time = time.time()
        solution_actions = self.get_steps(initial_state, goal_state)
        print(
            f"Execution time in seconds: {(time.time() - begin_time):.2f} | "
            f"Nodes generated: {self.nodes_generated} | "
            f"Nodes evaluated: {self.nodes_evaluated}"
        )
        return solution_actions


class ExampleAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):
        state = initial_state
        solution_actions = []
        while state != goal_state:
            legal_actions = self.get_legal_actions(state)
            action = legal_actions[random.randint(0, len(legal_actions) - 1)]
            solution_actions.append(action)
            state = self.apply_action(state, action)
        return solution_actions


class BFS(Algorithm):
    def get_steps(self, initial_state, goal_state):
        visitedStates = set()
        visitedStates.add(initial_state)
        queue = deque([(initial_state, [])])

        while queue:
            currentState, currentPath = queue.popleft()

            if currentState == goal_state:
                return currentPath

            legalActions = self.get_legal_actions(currentState)

            for action in legalActions:
                nextState = self.apply_action(currentState, action)

                if nextState not in visitedStates:
                    visitedStates.add(nextState)
                    queue.append((nextState, currentPath + [action]))

        return []


class BF(Algorithm):
    def get_steps(self, initial_state, goal_state):
        visited = set()
        priorityQueue = PriorityQueue()
        priorityQueue.put(
            (self.heuristic.get_evaluation(initial_state), initial_state, [])
        )

        while not priorityQueue.empty():
            _, currentState, currentPath = priorityQueue.get()

            if currentState in visited:
                continue

            visited.add(currentState)

            if currentState == goal_state:
                return currentPath

            legalActions = self.get_legal_actions(currentState)
            for action in legalActions:
                newState = self.apply_action(currentState, action)
                if newState not in visited:
                    newPath = currentPath + [action]
                    priorityQueue.put(
                        (self.heuristic.get_evaluation(newState), newState, newPath)
                    )

        return None


class AStar(Algorithm):
    def __init__(self, heuristic=None):
        super().__init__(heuristic)
        self.exploredStates = set()

    def get_steps(self, initial_state, goal_state):
        openSet = [(0, initial_state, [])]
        heapq.heapify(openSet)
        gValues = {initial_state: 0}

        while openSet:
            currentF, currentState, actions = heapq.heappop(openSet)
            if currentState == goal_state:
                return actions

            if currentState not in self.exploredStates:
                self.exploredStates.add(currentState)
                legalActions = self.get_legal_actions(currentState)

                for action in legalActions:
                    successorState = self.apply_action(currentState, action)

                    cost = gValues[currentState] + 1

                    if successorState not in gValues or cost < gValues[successorState]:
                        gValues[successorState] = cost
                        hValue = self.heuristic.get_evaluation(successorState)
                        fValue = cost + hValue
                        heapq.heappush(
                            openSet, (fValue, successorState, actions + [action])
                        )

        return None

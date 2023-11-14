from collections import deque
import heapq
import queue
import random
import time
from queue import Queue

import config


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


class BreadthFirstSearchAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):
        visited = set()
        visited.add(initial_state)
        queue = deque([(initial_state, [])])

        while queue:
            current_state, path = queue.popleft()
            if current_state == goal_state:
                return path

            legal_actions = self.get_legal_actions(current_state)
            for action in legal_actions:
                next_state = self.apply_action(current_state, action)
                if next_state not in visited:
                    visited.add(next_state)
                    queue.append((next_state, path + [action]))

        return []


class AstarAlgorithm(Algorithm):
    def __init__(self, heuristic=None):
        super().__init__(heuristic)

    def get_steps(self, initial_state, goal_state):
        open_set = [(self.heuristic.get_evaluation(initial_state), 0, initial_state)]
        closed_set = set()
        came_from = {}
        g_scores = {initial_state: 0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal_state:
                return self.reconstruct_path(came_from, initial_state, goal_state)

            closed_set.add(current)
            for action in self.get_legal_actions(current):
                neighbor = self.apply_action(current, action)
                tentative_g_score = g_scores[current] + 1

                if neighbor in closed_set or (
                    neighbor in g_scores and tentative_g_score >= g_scores[neighbor]
                ):
                    continue

                if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g_score
                    heapq.heappush(
                        open_set,
                        (
                            tentative_g_score + self.heuristic.get_evaluation(neighbor),
                            tentative_g_score,
                            neighbor,
                        ),
                    )

        return None

    def reconstruct_path(self, came_from, initial_state, goal_state):
        current = goal_state
        path = []
        while current != initial_state:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path


class BestFirstSearch(Algorithm):
    def __init__(self, heuristic=None):
        super().__init__(heuristic)
        self.memoization = {}

    def get_steps(self, initial_state, goal_state):
        self.nodes_evaluated = 0
        self.nodes_generated = 0

        open_set = queue.PriorityQueue()
        open_set.put((self.heuristic.get_evaluation(initial_state), initial_state))
        closed_set = set()

        while not open_set.empty():
            current_priority, current_state = open_set.get()

            if current_state == goal_state:
                return self.extract_solution_steps(initial_state, current_state)

            closed_set.add(current_state)
            legal_actions = self.get_legal_actions(current_state)

            for action in legal_actions:
                neighbor_state = self.apply_action(current_state, action)

                if (
                    neighbor_state not in closed_set
                    and neighbor_state not in self.memoization
                ):
                    priority = self.heuristic.get_evaluation(neighbor_state)
                    open_set.put((priority, neighbor_state))
                    self.memoization[neighbor_state] = priority
                    closed_set.add(neighbor_state)

                    self.nodes_generated += 1

            self.nodes_evaluated += 1

        # No solution found
        return []

    def extract_solution_steps(self, initial_state, goal_state):
        current_state = goal_state
        solution_actions = []

        while current_state != initial_state:
            for action in self.get_legal_actions(current_state):
                neighbor_state = self.apply_action(current_state, action)
                if neighbor_state == current_state:
                    solution_actions.append(action)
                    current_state = neighbor_state
                    break

        return solution_actions[::-1]

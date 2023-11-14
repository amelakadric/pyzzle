from collections import deque
import heapq
import queue
import random
import time
from queue import PriorityQueue, Queue

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


class BFS(Algorithm):
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


class BF(Algorithm):
    def get_steps(self, initial_state, goal_state):
        visited = set()
        priority_queue = PriorityQueue()
        priority_queue.put(
            (self.heuristic.get_evaluation(initial_state), initial_state, [])
        )

        while not priority_queue.empty():
            _, current_state, current_path = priority_queue.get()

            if current_state in visited:
                continue

            visited.add(current_state)

            if current_state == goal_state:
                return current_path

            legal_actions = self.get_legal_actions(current_state)
            for action in legal_actions:
                new_state = self.apply_action(current_state, action)
                if new_state not in visited:
                    new_path = current_path + [action]
                    priority_queue.put(
                        (self.heuristic.get_evaluation(new_state), new_state, new_path)
                    )

        return None


class AStar(Algorithm):
    def __init__(self, heuristic=None):
        super().__init__(heuristic)
        self.explored_states = set()

    def get_steps(self, initial_state, goal_state):
        open_set = [(0, initial_state, [])]  # Priority queue (f-value, state, actions)
        heapq.heapify(open_set)
        g_values = {initial_state: 0}

        while open_set:
            current_f, current_state, actions = heapq.heappop(open_set)
            if current_state == goal_state:
                return actions

            if current_state not in self.explored_states:
                self.explored_states.add(current_state)
                legal_actions = self.get_legal_actions(current_state)

                for action in legal_actions:
                    successor_state = self.apply_action(current_state, action)
                    cost = g_values[current_state] + 1

                    if (
                        successor_state not in g_values
                        or cost < g_values[successor_state]
                    ):
                        g_values[successor_state] = cost
                        h_value = self.heuristic.get_evaluation(successor_state)
                        f_value = cost + h_value
                        heapq.heappush(
                            open_set, (f_value, successor_state, actions + [action])
                        )

        return None

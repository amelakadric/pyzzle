from collections import deque
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
        queue = deque([(initial_state, [])])

        while queue:
            current_state, path = queue.popleft()
            if current_state == goal_state:
                return path

            visited.add(current_state)
            legal_actions = self.get_legal_actions(current_state)
            for action in legal_actions:
                next_state = self.apply_action(current_state, action)
                if next_state not in visited:
                    queue.append((next_state, path + [action]))

        return []


from queue import PriorityQueue


class BestFirstSearchAlgorithm(Algorithm):
    def __init__(self, heuristic=None):
        super().__init__(heuristic)

    def get_steps(self, initial_state, goal_state):
        begin_time = time.time()
        priority_queue = PriorityQueue()
        visited_states = set()

        # Inicijalni čvor
        initial_node = (self.heuristic.get_evaluation(initial_state), 0, initial_state)
        priority_queue.put(initial_node)

        while not priority_queue.empty():
            _, cost, current_state = priority_queue.get()

            if current_state == goal_state:
                print(
                    f"Execution time in seconds: {(time.time() - begin_time):.2f} | "
                    f"Nodes generated: {self.nodes_generated} | "
                    f"Nodes evaluated: {self.nodes_evaluated}"
                )
                return self.extract_solution_actions(initial_node, current_state)

            if current_state in visited_states:
                continue

            visited_states.add(current_state)
            legal_actions = self.get_legal_actions(current_state)

            for action in legal_actions:
                new_state = self.apply_action(current_state, action)
                if new_state not in visited_states:
                    heuristic_value = self.heuristic.get_evaluation(new_state)
                    new_node = (heuristic_value, cost + 1, new_state)
                    priority_queue.put(new_node)

                    self.nodes_generated += 1

            self.nodes_evaluated += 1

        # Ako ne pronađemo rešenje
        return None

    def extract_solution_actions(self, initial_node, final_state):
        solution_actions = []
        current_node = (self.heuristic.get_evaluation(final_state), 0, final_state)

        while current_node != initial_node:
            _, _, state = current_node
            legal_actions = self.get_legal_actions(state)

            for action in legal_actions:
                new_state = self.apply_action(state, action)
                if new_state == current_node[2]:
                    solution_actions.append(action)
                    break

            current_node = (self.heuristic.get_evaluation(state), 0, state)

        return solution_actions[::-1]

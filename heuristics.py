from state import get_pos_2d


class Heuristic:
    def get_evaluation(self, state):
        pass


class ExampleHeuristic(Heuristic):
    def get_evaluation(self, state):
        return 0

from config import N
class HammingHeuristic(Heuristic):
    def get_evaluation(self, state):
        h = 0
        for ident in range(N ** 2):
            goal_place_1d = ident - 1 if ident else N ** 2 - 1
            current_place_1d = state.index(ident)
            if current_place_1d != goal_place_1d:
                h += 1
        return h
class ManhattanHeuristic(Heuristic):
    def get_evaluation(self, state, goal_state):
        total_distance = 0
        for tile in range(len(state)):
            if state[tile] != 0:
                current_pos = get_pos_2d(state.index(tile))
                goal_pos = get_pos_2d(goal_state.index(tile))
                distance = abs(current_pos[0] - goal_pos[0]) + abs(current_pos[1] - goal_pos[1])
                total_distance += distance
        return total_distance

# goal_reader.py
from itertools import cycle
from .goal_generator import GoalGenerator

class GoalReader(GoalGenerator):
    """Yield goals from a text file forever (or up to iteration_count)."""

    def __init__(self, file_path):
        goals = [
            list(map(float, row.strip().split()))
            for row in open(file_path, "r") if row.strip()
        ]
        self._generator = cycle(goals)

    def generate_goal(self, max_num_of_trials=10000):
        return next(self._generator)


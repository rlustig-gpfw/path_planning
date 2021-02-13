import random
from typing import Tuple, Optional

import numpy as np
from attr import attrs


class Items(object):
    OPEN = 0
    OBSTACLE = 1
    ROBOT = 2
    GOAL = 3


@attrs(auto_attribs=True)
class Costmap(object):

    rows: int
    cols: int
    robot: Tuple[int, int]
    goal: Tuple[int, int]
    _data: 'Array[M,N]'
    _cell_size: int = 25

    @classmethod
    def create_map(
            cls,
            rows: int,
            cols: int,
            cell_size: int = 25,
            robot: Optional[Tuple[int, int]] = None,
            goal: Optional[Tuple[int, int]] = None
    ):
        if not robot:
            robot = [0, 0]

        if not goal:
            goal = [rows - 1, cols - 1]

        map = np.zeros((rows, cols), dtype=np.uint8)
        map[robot[0], robot[1]] = Items.ROBOT
        map[goal[0], goal[1]] = Items.GOAL

        print(f"Robot: {robot}")
        print(f"Goal: {goal}")

        return Costmap(rows, cols, robot, goal, map, cell_size)

    def get_data(self):
        return self._data

    def print(self):
        for r in range(0, self.rows):
            print()
            for c in range(0, self.cols):
                print(self._data[r, c], end=" ")
        print()

    def set_robot(self, robot_loc_rowcol: Tuple[int, int]):
        self._data[self.robot[0], self.robot[1]] = Items.OPEN
        self._data[robot_loc_rowcol[0], robot_loc_rowcol[1]] = Items.ROBOT

    def set_goal(self, goal_loc_rowcol: Tuple[int, int]):
        self._data[self.goal[0], self.goal[1]] = Items.OPEN
        self._data[goal_loc_rowcol[0], goal_loc_rowcol[1]] = Items.GOAL


def generate_random_rowcol(max_row: int, max_col: int) -> Tuple[int, int]:
    return random.randint(0, max_row - 1), random.randint(0, max_col - 1)


def generate_random_costmap(rows: int = 5, cols: int = 5, obstacle_percentage: float = 0.25):

    assert rows > 1 and cols > 1
    obstacles = np.random.rand(rows, cols) <= obstacle_percentage
    robot = generate_random_rowcol(rows, cols)
    goal = robot
    while robot == goal:
        goal = generate_random_rowcol(rows, cols)

    costmap = Costmap.create_map(rows, cols)
    data = costmap.get_data()
    data[obstacles] = Items.OBSTACLE

    print(f"New")
    print(f"Robot: {robot}")
    print(f"Goal: {goal}")

    costmap.set_robot(robot)
    costmap.set_goal(goal)

    return costmap


if __name__ == "__main__":
    # m = Costmap.create_map(7, 7, robot=(2, 2), goal=(5, 6))
    # m.print()
    costmap = generate_random_costmap()
    costmap.print()

import random
from typing import Tuple, Optional, Sequence, Dict, Any

import matplotlib.pyplot as plt
import numpy as np
from attr import attrs, attrib
from matplotlib.colors import LinearSegmentedColormap, Colormap, Normalize

from utils import clamp


class Colors:
    WHITE = (255, 255, 255)
    GRAY = (170, 170, 170)
    DARK_GRAY = (128, 128, 128)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    LIGHT_BLUE = (173, 209, 255)


class Items(object):
    OPEN = 0
    OBSTACLE = 1
    ROBOT = 2
    GOAL = 3

    CURRENT = 4
    VISITED = 5


ITEMS_TO_COLOR_MAPPING = {
    Items.OPEN: Colors.WHITE,
    Items.OBSTACLE: Colors.BLACK,
    Items.ROBOT: Colors.BLUE,
    Items.GOAL: Colors.GREEN,
    Items.CURRENT: Colors.LIGHT_BLUE,
    Items.VISITED: Colors.GRAY,
}


@attrs(auto_attribs=True)
class Costmap(object):

    rows: int
    cols: int
    robot: Tuple[int, int]
    goal: Tuple[int, int]

    _data: 'Array[M,N]'
    _cell_size: int = 25
    _items_to_colors_mapping: Optional[Dict[int, Tuple[int, int, int]]] = attrib(default=ITEMS_TO_COLOR_MAPPING)

    _fig: Any = attrib(init=False)
    _ax: plt.Axes = attrib(init=False)

    _cmap: Colormap = attrib(init=False)
    _norm: Normalize = attrib(init=False)

    def __attrs_post_init__(self):
        self._fig, self._ax = plt.subplots()
        self._cmap, self._norm = self._create_colormap()

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

        costmap = np.zeros((rows, cols), dtype=np.uint8)
        costmap[robot[0], robot[1]] = Items.ROBOT
        costmap[goal[0], goal[1]] = Items.GOAL

        print(f"Robot: {robot}")
        print(f"Goal: {goal}")

        return Costmap(
            rows=rows,
            cols=cols,
            robot=robot,
            goal=goal,
            data=costmap,
            cell_size=cell_size)

    def get_data(self):
        return self._data

    def get_neighbors(self, row: int, col: int) -> Optional[Sequence]:
        neighbors = []
        min_row = clamp(row - 1, 0, self.rows - 1)
        max_row = clamp(row + 1, 0, self.rows - 1)
        min_col = clamp(col - 1, 0, self.cols - 1)
        max_col = clamp(col + 1, 0, self.cols - 1)
        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                if r == row and c == col:
                    continue
                if self._data[r, c] not in [Items.OBSTACLE, Items.ROBOT, Items.VISITED]:
                    neighbors.append((r, c))
        return neighbors

    def set_robot(self, robot_loc_rowcol: Tuple[int, int]):
        self._data[self.robot[0], self.robot[1]] = Items.OPEN
        self.robot = robot_loc_rowcol
        self._data[robot_loc_rowcol[0], robot_loc_rowcol[1]] = Items.ROBOT

    def set_goal(self, goal_loc_rowcol: Tuple[int, int]):
        self._data[self.goal[0], self.goal[1]] = Items.OPEN
        self.goal = goal_loc_rowcol
        self._data[goal_loc_rowcol[0], goal_loc_rowcol[1]] = Items.GOAL

    def set_value(self, rowcol: Tuple[int, int], value: Items):
        self._data[rowcol[0], rowcol[1]] = value

    def _create_colormap(self):
        color_vals = list(self._items_to_colors_mapping.keys())
        colors = [np.array(c)/255. for c in self._items_to_colors_mapping.values()]

        norm = plt.Normalize(min(color_vals), max(color_vals))
        tuples = list(zip(map(norm, color_vals), colors))
        cmap = LinearSegmentedColormap.from_list("", tuples)
        return cmap, norm

    def print(self):
        for r in range(0, self.rows):
            print()
            for c in range(0, self.cols):
                print(self._data[r, c], end=" ")
        print()

    def draw(self):
        fig, ax = plt.subplots()
        ax.imshow(self._data, cmap=self._cmap, norm=self._norm)
        ax.axis('equal')
        ax.set_xlim(-0.5, self.cols - 0.5)
        ax.set_ylim(-0.5, self.rows - 0.5)
        plt.draw()
        plt.pause(0.05)


def generate_random_rowcol(max_row: int, max_col: int) -> Tuple[int, int]:
    return random.randint(0, max_row - 1), random.randint(0, max_col - 1)


def generate_random_costmap(rows: int = 10, cols: int = 10, obstacle_percentage: float = 0.25):

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
    costmap = generate_random_costmap(20, 30)
    costmap.print()

    print(costmap.get_neighbors(0, 0))
    costmap.draw()

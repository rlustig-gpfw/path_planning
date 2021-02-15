import random
from typing import Tuple, Optional, Sequence, Dict

import numpy as np
from PIL import Image, ImageDraw
from attr import attrs

from utils import clamp


class Colors:
    WHITE = (255, 255, 255)
    GRAY = (150, 150, 150)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)


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
    _items_to_colors_mapping: Optional[Dict] = None

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

        items_to_colors_mapping = cls.get_item_to_colors_mapping()

        return Costmap(
            rows=rows,
            cols=cols,
            robot=robot,
            goal=goal,
            data=map,
            cell_size=cell_size,
            items_to_colors_mapping=items_to_colors_mapping)

    def get_data(self):
        return self._data

    def get_neighbors(self, row: int, col: int) -> Optional[Sequence]:
        neighbors = []
        min_row = clamp(row - 1, 0, self.rows - 1)  #max(min(row - 1, self.rows + 1), self.rows + 1)  #row - 1 if row > 0 else 0
        max_row = clamp(row + 1, 0, self.rows - 1)
        min_col = clamp(col - 1, 0, self.cols - 1)
        max_col = clamp(col + 1, 0, self.cols - 1)
        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                if r == row and c == col:
                    continue
                if self._data[r, c] == Items.OPEN:
                    neighbors.append((r, c))
        return neighbors

    def set_robot(self, robot_loc_rowcol: Tuple[int, int]):
        self._data[self.robot[0], self.robot[1]] = Items.OPEN
        self._data[robot_loc_rowcol[0], robot_loc_rowcol[1]] = Items.ROBOT

    def set_goal(self, goal_loc_rowcol: Tuple[int, int]):
        self._data[self.goal[0], self.goal[1]] = Items.OPEN
        self._data[goal_loc_rowcol[0], goal_loc_rowcol[1]] = Items.GOAL

    def set_value(self, rowcol: Tuple[int, int], value: Items):
        self._data[rowcol[0], rowcol[1]] = value

    @classmethod
    def get_item_to_colors_mapping(cls):
        items_to_colors_mapping = {
            Items.OPEN: Colors.WHITE,
            Items.OBSTACLE: Colors.BLACK,
            Items.ROBOT: Colors.BLUE,
            Items.GOAL: Colors.GREEN
        }
        return items_to_colors_mapping

    def print(self):
        for r in range(0, self.rows):
            print()
            for c in range(0, self.cols):
                print(self._data[r, c], end=" ")
        print()

    def draw(self):
        if not self._items_to_colors_mapping:
            self._items_to_colors_mapping = Costmap.get_item_to_colors_mapping()

        image_size = self.cols * self._cell_size, self.rows * self._cell_size
        grid = Image.new("RGB", image_size, Colors.WHITE)
        draw = ImageDraw.Draw(grid)

        for r in range(0, self.rows):
            for c in range(0, self.cols):
                # Find top left and bottom right box corners
                top_left = c * self._cell_size, r * self._cell_size
                btm_right = (c + 1) * self._cell_size, (r + 1) * self._cell_size
                box = (top_left, btm_right)
                # Draw box and fill with desired color
                fill_color = self._items_to_colors_mapping[self._data[r, c]]
                draw.rectangle(box, fill=fill_color, outline=Colors.GRAY)
        del draw
        grid.show()


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

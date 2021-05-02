import random
from typing import Tuple, Optional, Sequence, Dict, Any, List, Union

import imageio
import matplotlib.pyplot as plt
import numpy as np
from attr import attrs, attrib
from matplotlib.colors import Colormap, Normalize

from utils import clamp


class Colors:
    WHITE = (255, 255, 255)
    GRAY = (170, 170, 170)
    DARK_GRAY = (128, 128, 128)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    LIGHT_BLUE = (173, 209, 255)
    PURPLE = (122, 37, 118)


class Items(object):
    OPEN = 0
    OBSTACLE = 1
    ROBOT = 2
    GOAL = 3

    CURRENT = 4
    VISITED = 5
    PARENT = 6


ITEMS_TO_COLOR_MAPPING = {
    Items.OPEN: Colors.WHITE,
    Items.OBSTACLE: Colors.BLACK,
    Items.ROBOT: Colors.BLUE,
    Items.GOAL: Colors.GREEN,
    Items.CURRENT: Colors.LIGHT_BLUE,
    Items.VISITED: Colors.GRAY,
    Items.PARENT: Colors.PURPLE
}


@attrs(auto_attribs=True, slots=True, frozen=True)
class Location:
    x: int
    y: int

    def __sub__(self, other: 'Location'):
        return Location(self.x - other.x, self.y - other.y)

    def __str__(self):
        return f"(x={self.x}, y={self.y})"


@attrs(auto_attribs=True)
class Costmap(object):

    rows: int
    cols: int
    robot: Location
    goal: Location

    _data: 'Array[M,N]'
    _items_to_colors_mapping: Optional[Dict[int, Tuple[int, int, int]]] = attrib(default=ITEMS_TO_COLOR_MAPPING)

    _fig: Any = attrib(init=False)
    _ax: plt.Axes = attrib(init=False)

    _cmap: Colormap = attrib(init=False)
    _norm: Normalize = attrib(init=False)

    @classmethod
    def create_map(
            cls,
            rows: int,
            cols: int,
            robot: Optional[Location] = None,
            goal: Optional[Location] = None
    ):
        if not robot:
            robot = Location(0, 0)

        if not goal:
            goal = Location(cols - 1, rows - 1)

        costmap = np.zeros((rows, cols), dtype=np.uint8)
        costmap[robot.y, robot.x] = Items.ROBOT
        costmap[goal.y, goal.x] = Items.GOAL

        return Costmap(
            rows=rows,
            cols=cols,
            robot=robot,
            goal=goal,
            data=costmap)

    def get_data(self) -> 'Array[M,N]':
        return self._data

    def get_value(self, location) -> Union[int, float]:
        return self._data[location.y, location.x]

    def get_open_neighbors(self, loc: Location) -> Optional[Sequence[Location]]:
        col = loc.x
        row = loc.y
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
                    neighbors.append(Location(x=c, y=r))
        return neighbors

    def set_robot(self, robot: Location) -> None:
        if self.get_value(robot) != Items.GOAL:
            self._data[self.robot.y, self.robot.x] = Items.OPEN
        self.robot = robot
        self._data[robot.y, robot.x] = Items.ROBOT

    def set_goal(self, goal: Location) -> None:
        if self.get_value(goal) != Items.ROBOT:
            self._data[self.goal.y, self.goal.x] = Items.OPEN
        self.goal = goal
        self._data[goal.y, goal.x] = Items.GOAL

    def set_value(self, loc: Location, value: Items) -> None:
        self._data[loc.y, loc.x] = value

    def print(self) -> None:
        for r in range(0, self.rows):
            print()
            for c in range(0, self.cols):
                print(self._data[r, c], end=" ")
        print()

    def draw(self, show: bool = True) -> 'Array[M,N,3]':
        display_image = self._colorize_costmap()
        if show:
            fig, ax = plt.subplots()
            ax.imshow(display_image)
            ax.axis('off')
            plt.draw()
            plt.pause(0.05)

        return display_image

    def _colorize_costmap(self) -> 'Array[M,N,3]':
        colorized_costmap = np.zeros((self.rows, self.cols, 3), dtype=np.uint8)
        for item, color in self._items_to_colors_mapping.items():
            mask = self._data == item
            colorized_costmap[mask] = color

        return colorized_costmap


@attrs(auto_attribs=True, slots=True)
class EasyGIFWriter(object):
    _file_path: str
    _frames_per_second: int = 18
    _frame_list: List['Array[M,N,3]'] = list()
    _scale_factor: int = 1
    _hold_last_frame_time: float = 2

    def __enter__(self):
        return self

    def write(self, image):
        self._frame_list.append(image)

    def __exit__(self, exc_type, exc_val, exc_tb):
        assert len(self._frame_list) > 0, "The number of GIF frames is 0"
        if self._scale_factor != 1:
            frame_buffer = []
            for im in self._frame_list:
                # NN upsampling only - doesn't require other libs
                upscaled_im = im.repeat(self._scale_factor, axis=0).repeat(self._scale_factor, axis=1)
                frame_buffer.append(upscaled_im)
        else:
            frame_buffer = self._frame_list

        # Add frames to the end of the gif, if desired
        num_last_frames = self._hold_last_frame_time * self._frames_per_second
        if num_last_frames > 1:
            for _ in range(1, int(num_last_frames)):
                frame_buffer.append(frame_buffer[-1])

        # Write out gif
        with open(self._file_path, 'wb') as video_file:
            imageio.mimwrite(video_file, frame_buffer, "GIF", fps=self._frames_per_second)


def generate_random_location(max_row: int, max_col: int) -> Location:
    return Location(x=random.randint(0, max_col - 1), y=random.randint(0, max_row - 1))


def generate_random_costmap(rows: int = 10, cols: int = 10, obstacle_percentage: float = 0.25):

    assert rows > 1 and cols > 1
    obstacles = np.random.rand(rows, cols) <= obstacle_percentage
    robot = generate_random_location(rows, cols)
    goal = robot
    while robot == goal:
        goal = generate_random_location(rows, cols)

    costmap = Costmap.create_map(rows, cols)
    data = costmap.get_data()
    data[obstacles] = Items.OBSTACLE

    costmap.set_robot(robot)
    costmap.set_goal(goal)

    return costmap


def generate_vertical_wall_costmap(rows: int = 10, cols: int = 10):
    assert rows > 1 and cols > 1
    # Find
    wall_rows = list(range(rows))
    # wall_rows.remove(rows // 2)  # Remove middle element
    wall_rows.remove(0)
    wall_col = cols // 2

    robot = Location(0, 0)
    goal = Location(cols - 1, rows - 1)

    costmap = Costmap.create_map(rows, cols)
    data = costmap.get_data()
    for row in wall_rows:
        data[row, wall_col] = Items.OBSTACLE

    costmap.set_robot(robot)
    costmap.set_goal(goal)

    return costmap


if __name__ == "__main__":
    costmap = generate_random_costmap(20, 30)
    costmap.print()

    print(costmap.get_open_neighbors(Location(0, 0)))
    costmap.draw()

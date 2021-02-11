from typing import Tuple, Optional, Sequence

import numpy as np
from attr import attrs


class Items(object):
    OPEN = 0
    OBSTACLE = 1
    ROBOT = 2
    GOAL = 3


@attrs(auto_attribs=True)
class Map(object):

    rows: int
    cols: int
    robot: Tuple[int, int]
    goal: Tuple[int, int]
    map: 'Array[M,N]'
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

        return Map(rows, cols, robot, goal, map, cell_size)

    def print(self):
        for r in range(0, self.rows):
            print()
            for c in range(0, self.cols):
                print(self.map[r, c], end=" ")


if __name__ == "__main__":
    m = Map.create_map(7, 7, robot=(2, 2), goal=(5, 6))
    m.print()




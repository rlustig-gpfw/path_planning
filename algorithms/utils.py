from typing import Tuple


Color = Tuple[int, int, int]


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


def clamp(num, smallest, largest):
    return max(smallest, min(num, largest))

import numpy as np

from algorithms.costmap import Costmap, Location
from algorithms.utils import Items


def test_creation():
    costmap = Costmap.create_map(rows=100, cols=100)
    assert costmap.rows == 100
    assert costmap.cols == 100
    expected_costmap = np.zeros((100, 100), dtype=np.uint8)
    expected_costmap[0, 0] = Items.ROBOT
    expected_costmap[-1, -1] = Items.GOAL
    assert np.all(costmap.get_data() == expected_costmap)

    cols = 45
    rows = 50
    robot = Location(5, 5)
    goal = Location(42, 42)
    costmap = Costmap.create_map(rows=rows, cols=cols, robot=robot, goal=goal)
    expected_costmap = np.zeros((rows, cols), dtype=np.uint8)
    expected_costmap[robot.y, robot.x] = Items.ROBOT
    expected_costmap[goal.y, goal.x] = Items.GOAL
    assert np.all(costmap.get_data() == expected_costmap)


def test_set_values():
    costmap = Costmap.create_map(rows=100, cols=100)
    loc1 = Location(5, 10)
    costmap.set_value(loc1, Items.CURRENT)
    assert costmap.get_value(loc1) == Items.CURRENT

    loc2 = Location(-1, -1)
    costmap.set_value(loc2, Items.VISITED)
    assert costmap.get_value(loc2) == Items.VISITED


def test_set_robot_goal():
    costmap = Costmap.create_map(rows=10, cols=10)
    robot = Location(5, 5)
    costmap.set_robot(robot)
    assert costmap.get_value(robot) == Items.ROBOT
    goal = Location(1, 9)
    costmap.set_goal(goal)
    assert costmap.get_value(goal) == Items.GOAL

    robot = Location(0, 0)
    goal = Location(9, 9)
    costmap = Costmap.create_map(rows=10, cols=10, robot=robot, goal=goal)
    assert costmap.get_value(robot) == Items.ROBOT
    assert costmap.get_value(goal) == Items.GOAL


def test_get_open_neighbors():
    costmap = Costmap.create_map(rows=10, cols=10, robot=Location(4, 4), goal=Location(4, 5))
    curr = Location(6, 6)
    visited = Location(5, 6)
    costmap.set_value(curr, Items.CURRENT)
    costmap.set_value(visited, Items.VISITED)

    center = Location(5, 5)
    open_neighbors = costmap.get_open_neighbors(center)
    expected_open_neighbors = {
        Location(4, 5),
        Location(4, 6),
        Location(5, 4),
        Location(6, 4),
        Location(6, 5),
        Location(6, 6),
    }
    assert set(open_neighbors) == expected_open_neighbors


if __name__ == '__main__':
    test_creation()
    test_set_values()
    test_set_robot_goal()
    test_get_open_neighbors()

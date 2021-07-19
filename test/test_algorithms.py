import numpy as np

from algorithms.astar import AStar, AStarHeuristics
from algorithms.breadth_first_search import BFS
from algorithms.costmap import Costmap, generate_random_costmap, Location
from algorithms.depth_first_search import DFS
from algorithms.utils import Items


def create_test_costmap() -> Costmap:
    np.random.seed(42)
    costmap = generate_random_costmap(10, 10, obstacle_percentage=.20)
    robot = Location(2, 3)
    goal = Location(8, 7)
    costmap.set_robot(robot)
    costmap.set_goal(goal)
    return costmap


def create_test_costmap_with_wall() -> Costmap:
    rows = 10
    cols = 10
    wall_rows = list(range(rows))
    wall_rows.remove(0)
    wall_col = cols // 2

    robot = Location(2, 1)
    goal = Location(7, 8)

    costmap = Costmap.create_map(rows, cols)
    data = costmap.get_data()
    for row in wall_rows:
        data[row, wall_col] = Items.OBSTACLE

    costmap.set_robot(robot)
    costmap.set_goal(goal)

    return costmap


def test_bfs():
    costmap = create_test_costmap()
    bfs = BFS(costmap)
    while True:
        path = bfs.step()
        if path:
            break

    expected_path = [
        Location(x=3, y=4),
        Location(x=4, y=5),
        Location(x=5, y=6),
        Location(x=6, y=7),
        Location(x=7, y=8),
        Location(x=8, y=7)
    ]
    assert expected_path == path


def test_dfs():
    costmap = create_test_costmap()
    dfs = DFS(costmap)
    while True:
        path = dfs.step()
        if path:
            break

    expected_path = [
        Location(x=3, y=4),
        Location(x=4, y=5),
        Location(x=5, y=6),
        Location(x=6, y=7),
        Location(x=7, y=8),
        Location(x=8, y=8),
        Location(x=9, y=8),
        Location(x=8, y=7)
    ]
    assert expected_path == path


def test_astar():
    costmap = create_test_costmap_with_wall()

    expected_paths = dict()
    expected_paths[AStarHeuristics.manhattan] = [
        Location(x=3, y=1),
        Location(x=4, y=1),
        Location(x=5, y=0),
        Location(x=6, y=1),
        Location(x=7, y=2),
        Location(x=7, y=3),
        Location(x=7, y=4),
        Location(x=7, y=5),
        Location(x=7, y=6),
        Location(x=7, y=7),
        Location(x=7, y=8)
    ]
    expected_paths[AStarHeuristics.euclidean] = [
        Location(x=3, y=1),
        Location(x=4, y=1),
        Location(x=5, y=0),
        Location(x=6, y=1),
        Location(x=7, y=2),
        Location(x=7, y=3),
        Location(x=7, y=4),
        Location(x=7, y=5),
        Location(x=7, y=6),
        Location(x=7, y=7),
        Location(x=7, y=8)
    ]
    expected_paths[AStarHeuristics.chebyshev] = [
        Location(x=3, y=1),
        Location(x=4, y=1),
        Location(x=5, y=0),
        Location(x=6, y=1),
        Location(x=6, y=2),
        Location(x=6, y=3),
        Location(x=6, y=4),
        Location(x=6, y=5),
        Location(x=6, y=6),
        Location(x=6, y=7),
        Location(x=7, y=8)
    ]

    heuristics = [
        AStarHeuristics.manhattan,
        AStarHeuristics.euclidean,
        AStarHeuristics.chebyshev
    ]

    for heuristic in heuristics:
        astar = AStar(costmap, heuristic)
        while True:
            path = astar.step()
            if path:
                break

        assert expected_paths[heuristic] == path


if __name__ == '__main__':
    test_bfs()
    test_dfs()
    test_astar()

from queue import PriorityQueue
from typing import Dict, Tuple, Callable, Optional, Sequence

from attr import attrs, attrib

from costmap import Costmap, Items, generate_random_costmap, EasyGIFWriter, generate_vertical_wall_costmap, Location


class AStarHeuristics:
    @staticmethod
    def manhattan(a: Location, b: Location):
        diff = a - b
        return abs(diff.x) + abs(diff.y)

    @staticmethod
    def euclidean(a: Location, b: Location):
        diff = a - b
        return (diff.x ** 2 + diff.y ** 2) ** 0.5

    @staticmethod
    def chebyshev(a: Location, b: Location):
        # Explores more as it searches for the goal
        diff = a - b
        dx = abs(diff.x)
        dy = abs(diff.y)
        return dx + dy - 2 * min(dx, dy)


@attrs(auto_attribs=True)
class AStar(object):
    _costmap: Costmap
    _heuristic: Callable[[Location, Location], float] = AStarHeuristics.euclidean

    _parent_map: Dict[Tuple, Tuple] = attrib(init=False, factory=dict)
    _queue: PriorityQueue = attrib(init=False, factory=lambda: PriorityQueue())
    _cost_so_far: Dict[Location, float] = attrib(init=False, factory=dict)

    def __attrs_post_init__(self):
        # Append robot position as the starting node
        self._queue.put((0, self._costmap.robot))
        self._goal = self._costmap.goal
        self._cost_so_far[self._costmap.robot] = 0

    def _compute_movement_cost(self, from_loc: Location, to_loc: Location) -> float:
        dist_cost = 1

        # Add small penalty to diagonal movements for better looking paths:
        # https://www.redblobgames.com/pathfinding/a-star/implementation.html
        penalty = 0
        (x1, y1) = from_loc.x, from_loc.y
        (x2, y2) = to_loc.x, to_loc.y
        if (x1 + y1) % 2 == 0 and x2 != x1:
            penalty = 1
        if (x1 + y1) % 2 == 1 and y2 != y1:
            penalty = 1
        return dist_cost + 0.001 * penalty

    def step(self) -> Optional[Sequence[Location]]:

        if self._queue.qsize() == 0:
            raise Exception("Path does not exist!")

        current_pos: Location = self._queue.get()[1]
        current_value = self._costmap.get_value(current_pos)

        if current_pos == self._goal:
            # Create and return path
            path = []
            curr = current_pos
            while curr != self._costmap.robot:
                if curr != self._costmap.goal:
                    self._costmap.set_value(curr, Items.PARENT)
                path.append(curr)
                curr = self._parent_map[curr]
            path.reverse()
            return path

        # Mark position as visited (closed list)
        if current_value != Items.ROBOT:
            self._costmap.set_value(current_pos, Items.VISITED)

        # Add neighbors to list, add to parent list
        neighbors = self._costmap.get_open_neighbors(current_pos)
        for n in neighbors:
            # Cost to move from current position to neighbot
            dist_cost = self._compute_movement_cost(current_pos, n)

            # Update base cost with cost to move to neighbor
            new_cost = self._cost_so_far[current_pos] + dist_cost

            if n not in self._cost_so_far or new_cost < self._cost_so_far[n]:
                # Save or update current cost
                self._cost_so_far[n] = new_cost
                # f = g + h
                priority = new_cost + self._heuristic(n, self._costmap.goal)
                # Add to queue
                self._queue.put((priority, n))
                # Save parents-to-child map so that the path can be extracted
                self._parent_map[n] = current_pos

                # Mark costmap value for visualization
                if self._costmap.get_value(n) != Items.GOAL:
                    self._costmap.set_value(n, Items.CURRENT)

        return None


if __name__ == "__main__":
    costmap = generate_random_costmap(20, 30, 0.4)
    # costmap = generate_vertical_wall_costmap(rows=10)
    costmap.draw()

    with EasyGIFWriter("astar.gif", scale_factor=25) as gif_writer:
        astar = AStar(costmap)
        while True:
            path = astar.step()
            im = costmap.draw(show=False)
            gif_writer.write(im)
            if path:
                break

    print(path)

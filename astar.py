from queue import PriorityQueue
from typing import Dict, Tuple, List

from attr import attrs, attrib

from costmap import Costmap, Items, generate_random_costmap, EasyGIFWriter, generate_vertical_wall_costmap


# TODO
# class AStarHueristics:
#     @staticmethod
#     def manhattan(node1, node2):
#         return


def heuristic(a, b) -> float:
    x1, y1 = a
    x2, y2 = b
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


@attrs(auto_attribs=True)
class AStar(object):
    # f = g + h
    # f = total cost of node
    # g = dist between current node and start node
    # h = heuristic - est dist from current node to end node
    _costmap: Costmap

    _parent_map: Dict[Tuple, Tuple] = attrib(init=False, factory=dict)
    _queue: PriorityQueue = attrib(init=False, factory=lambda: PriorityQueue())

    _cost_so_far: Dict[Tuple[int, int], float] = attrib(init=False, factory=dict)

    def __attrs_post_init__(self):
        # Append robot position as the starting node
        self._queue.put((0, self._costmap.robot))
        self._goal = self._costmap.goal
        self._cost_so_far[self._costmap.robot] = 0

    def step(self):

        if self._queue.qsize() == 0:
            raise Exception("Path does not exist!")

        _, current_pos = self._queue.get()
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
        neighbors = self._costmap.get_open_neighbors(current_pos[0], current_pos[1])
        for n in neighbors:
            dist_cost = 1.
            if (current_pos[0] - n[0]) == 0 or (current_pos[1] - n[1]) == 0:
                dist_cost = 2 ** 0.5

            new_cost = self._cost_so_far[current_pos] + dist_cost

            if n not in self._cost_so_far or new_cost < self._cost_so_far[n]:
                self._cost_so_far[n] = new_cost
                # f = g + h
                priority = new_cost + heuristic(n, self._costmap.goal)
                # Add to queue
                self._queue.put((priority, n))
                # Save parents-to-child map so that the path can be extracted
                self._parent_map[n] = current_pos

                # Mark costmap value so that it can be drawn
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

from queue import PriorityQueue
from typing import Dict, Tuple, List

from attr import attrs, attrib

from costmap import Costmap, Items, generate_random_costmap, EasyGIFWriter, generate_vertical_wall_costmap


# TODO
# class AStarHueristics:
#     @staticmethod
#     def manhattan(node1, node2):
#         return


@attrs(auto_attribs=True, frozen=True, slots=True)
class CellData:
    position: Tuple[int, int]
    costmap_value: int
    f: float = 0.
    g: float = 0.


@attrs(auto_attribs=True, frozen=True, order=False)
class PriorityCell(object):
    priority: float  # Any comparable data type
    data: CellData

    def __lt__(self, other):
        """ Implementing "less-than" allows you to use this object in a priority queue.
        :param other: Another Observation
        :return: True if this timestamp is less than the other.
        """
        return self.priority < other.priority


@attrs(auto_attribs=True)
class AStar(object):
    # f = g + h
    # f = total cost of node
    # g = dist between current node and start node
    # h = heuristic - est dist from current node to end node
    _costmap: Costmap

    _parent_map: Dict[Tuple, Tuple] = attrib(init=False, factory=dict)
    _queue: PriorityQueue = attrib(init=False, factory=lambda: PriorityQueue())
    _loc_cell_map: Dict[Tuple[int, int], CellData] = attrib(init=False, factory=dict)

    def __attrs_post_init__(self):
        # Append robot position as the starting node
        cell_data = CellData(position=self._costmap.robot, costmap_value=0)
        self._queue.put(PriorityCell(priority=0, data=cell_data))
        self._goal = self._costmap.goal

    def _compute_cost(self, current_costs, current_position):
        p_r, p_c = current_position
        g = current_costs.g + 1
        h = (self._costmap.goal[0] - p_r) ** 2 + (self._costmap.goal[1] - p_c) ** 2
        f = g + h
        return f, g

    def step(self):

        # VISITED = closed list
        # CURRENT = open list

        if self._queue.qsize() == 0:
            raise Exception("Path does not exist!")

        cell: PriorityCell = self._queue.get()
        cell_data = cell.data
        current_pos = cell_data.position
        current_value = self._costmap.get_data()[current_pos[0], current_pos[1]]

        if current_value == Items.GOAL:
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

            neighbor_value = self._costmap.get_data()[n[0], n[1]]

            # Shortcut to goal
            if n == self._costmap.goal:
                neighbor_cell_data = CellData(
                    position=n,
                    costmap_value=0,
                    f=-1
                )
                self._queue.put(PriorityCell(priority=-1, data=neighbor_cell_data))
                self._parent_map[n] = current_pos
                return

            # Compute cost
            f, g = self._compute_cost(cell_data, n)

            neighbor_cell_data = self._loc_cell_map.get(n, None)

            # Skip this neighbor if:
            #  - we've seen this cell before and
            #  - the current computed cost is greater than the previously computed cost
            if neighbor_value == Items.CURRENT and neighbor_cell_data and g > neighbor_cell_data.g:
                continue

            neighbor_cell_data = CellData(
                position=n,
                costmap_value=g,
                f=f,
                g=g
            )

            # Mark as current (open list)
            self._costmap.set_value(n, Items.CURRENT)
            # Add to queue
            self._queue.put(PriorityCell(priority=f, data=neighbor_cell_data))
            # Add to hashmap
            self._loc_cell_map[n] = neighbor_cell_data

            # Save parents-to-child map so that the path can be extracted
            self._parent_map[n] = current_pos

        return None


if __name__ == "__main__":
    costmap = generate_random_costmap(20, 30, 0.4)
    # costmap = generate_vertical_wall_costmap(rows=20)
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

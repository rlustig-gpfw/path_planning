from typing import Tuple, Dict, List, Sequence, Optional

from attr import attrs, attrib

from algorithms.costmap import Costmap, generate_random_costmap, EasyGIFWriter, Location
from algorithms.utils import Items


@attrs(auto_attribs=True)
class DFS(object):
    _costmap: Costmap

    _parent_map: Dict[Tuple, Tuple] = attrib(init=False, factory=dict)
    _stack: List[Location] = attrib(init=False, factory=list)

    def __attrs_post_init__(self):
        # Append robot position as the starting node
        self._stack.append(self._costmap.robot)
        self._goal = self._costmap.goal

    def step(self) -> Optional[Sequence[Location]]:

        if len(self._stack) == 0:
            raise Exception("Path does not exist!")

        current_pos = self._stack.pop()
        current_value = self._costmap.get_value(current_pos)

        if current_value == Items.GOAL:
            # Create and return path
            path = []
            curr = current_pos
            while curr != self._costmap.robot:
                if curr != self._goal:
                    self._costmap.set_value(curr, Items.PARENT)
                path.append(curr)
                curr = self._parent_map[curr]
            path.reverse()
            return path

        # Mark visited node
        if current_value != Items.ROBOT:
            self._costmap.set_value(current_pos, Items.VISITED)

        # Add neighbors to list, add to parent list
        neighbors = self._costmap.get_open_neighbors(current_pos)
        for n in neighbors:
            if self._costmap.get_value(n) == Items.CURRENT:
                continue

            if n != self._goal:
                self._costmap.set_value(n, Items.CURRENT)
            self._stack.append(n)
            # Save parents-to-child map so that the path can be extracted
            self._parent_map[n] = current_pos

        return None


if __name__ == "__main__":
    costmap = generate_random_costmap(20, 30)
    costmap.draw()

    with EasyGIFWriter("dfs.gif", scale_factor=25) as gif_writer:
        dfs = DFS(costmap)
        while True:
            path = dfs.step()
            im = costmap.draw()
            gif_writer.write(im)
            if path:
                break

    print(path)

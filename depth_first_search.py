from typing import Tuple, Dict, List

from attr import attrs, attrib

from costmap import Costmap, Items, generate_random_costmap


@attrs(auto_attribs=True)
class DFS(object):
    _costmap: Costmap

    _parent_map: Dict[Tuple, Tuple] = attrib(init=False, factory=dict)
    _stack: List[Tuple] = attrib(init=False, factory=list)

    def __attrs_post_init__(self):
        # Append robot position as the starting node
        self._stack.append(self._costmap.robot)
        self._goal = self._costmap.goal

    def step(self):

        if len(self._stack) == 0:
            raise Exception("Path does not exist!")

        current_pos = self._stack.pop()
        current_value = self._costmap.get_data()[current_pos[0], current_pos[1]]

        if current_value == Items.GOAL:
            # Create and return path
            path = []
            curr = current_pos
            while curr is not self._costmap.robot:
                path.append(curr)
                curr = self._parent_map[curr]
            path.reverse()
            return path

        # Mark visited node
        if current_value != Items.ROBOT:
            self._costmap.set_value(current_pos, Items.VISITED)

        # Add neighbors to list, add to parent list
        neighbors = self._costmap.get_neighbors(current_pos[0], current_pos[1])
        for n in neighbors:
            self._stack.append(n)
            # Save parents-to-child map so that the path can be extracted
            self._parent_map[n] = current_pos

        return None


if __name__ == "__main__":
    costmap = generate_random_costmap(8, 8)
    costmap.draw()

    dfs = DFS(costmap)
    while True:
        path = dfs.step()
        costmap.draw()
        if path:
            break

    print(path)

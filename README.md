# Search and Path Planning
Search and path planning algorithms with visualizations

## Algorithms

Within the `algorithms` folder, there are the following search-based algorithms:

- Depth First Search (DFS)
- Breadth First Search (BFS)
- A*
  - Includes the option to use different heuristic cost functions such as:
    - manhattan
    - euclidean
    - chebyshev

The following files are used to support the search and path planning visualization:

- `costmap.py` - A `Costmap` class that provides the ability to:
  - Create a grid with placement of a robot, goal, and obstacles
  - Manipulate the values of the costmap as the search algorithms move around the map
  - Draw the costmap and display as an image
This class also includes a random `Costmap` generator function as well a
context-managed `EasyGIFWriter` class to write out a list of images as a GIF.

### Example Outputs

#### Depth First Search (DFS)
![Depth First Search](gifs/dfs.gif)

#### Breadth First Search (BFS)
![Breadth First Search](gifs/bfs.gif)

#### A* Search
![A*](gifs/astar.gif)

### Example A* heuristics

#### Map 1
| Euclidean | Manhattan | Chebyshev|
|-----------|-----------|----------|
| <img src="https://user-images.githubusercontent.com/7671719/116958441-81806000-ac4f-11eb-83b9-847df624493c.gif" width=250></img> | <img src="https://user-images.githubusercontent.com/7671719/116958443-81806000-ac4f-11eb-85a8-0efbdb282410.gif" width=250></img> | <img src="https://user-images.githubusercontent.com/7671719/116958440-80e7c980-ac4f-11eb-8e5d-f4a83583ad1c.gif" width=250></img> |

#### Map 2
| Euclidean | Manhattan | Chebyshev|
|-----------|-----------|----------|
|  <img src="https://user-images.githubusercontent.com/7671719/116960365-0326bc80-ac55-11eb-9aec-4544c617eccf.gif" width=250>></img> | <img src="https://user-images.githubusercontent.com/7671719/116960368-03bf5300-ac55-11eb-974c-446846c926aa.gif" width=250>></img> | <img src="https://user-images.githubusercontent.com/7671719/116960364-01f58f80-ac55-11eb-8516-f8a59dada599.gif" width=250></img> |

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
![Breadth First Search](gifs/dfs.gif)

#### A* Search
![A*](gifs/astar.gif)

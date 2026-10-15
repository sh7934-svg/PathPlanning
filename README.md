# PathPlanning

An animated A* path planner on a grid, written in Python with matplotlib. The robot has a
**heading**, so A* searches over position *and* direction: turning in place costs as much
as moving forward, and the planner finds the path with the fewest total actions.

## Requirements

- Python 3
- `numpy`
- `matplotlib`

```bash
pip install numpy matplotlib
```

## Usage

```bash
python3 visual.py
```

A window opens and animates the search. When the goal is reached, the explored cells are
cleared and the final path is drawn with an arrow showing the robot's heading at each step.
The terminal prints the result:

```
Found Path. Total cost was 19.0
```

## How it works

### State and actions

Each search node is a `(row, col, direction)` state. Directions are numbered
`0` up, `1` right, `2` down, `3` left. From any state the robot can:

| Action        | Result                                   | Cost |
|---------------|------------------------------------------|------|
| Move forward  | One cell in the current direction        | 1    |
| Turn right    | Same cell, direction `(d + 1) % 4`       | 1    |
| Turn left     | Same cell, direction `(d - 1) % 4`       | 1    |

Forward moves are skipped if they would leave the grid or enter an obstacle.

### Search

- **g** is the number of actions taken so far.
- **h** is the straight-line (Euclidean) distance from the cell to the goal.
- **f = g + h** orders the open list, which is a `heapq` priority queue. A counter breaks
  ties so nodes are never compared directly.
- When a cheaper route to a state already in the open list is found, that node's cost and
  parent are updated and the heap is re-heapified.
- The goal is reached when the robot is on the goal cell, in any direction.
- The path is rebuilt by following `parent` links back from the goal.

### Colors

| Color  | Meaning                         |
|--------|---------------------------------|
| Black  | Obstacle                        |
| Green  | Start                           |
| Red    | Goal                            |
| Grey   | Open list (discovered)          |
| Blue   | Closed list (expanded)          |
| Yellow | Final path (with heading arrows)|

## Customizing the map

Everything is set near the bottom of `visual.py`. Positions are `(row, col)`, with `(0, 0)`
at the top-left.

```python
target = (0, 7)        # goal cell
start = (9, 0)         # start cell (robot starts facing up)
obstacles = [(0, 0), (1, 1), ...]
map = np.zeros((10, 8))  # grid size: rows, cols
```

If you change the grid size, also update `bounds = (10, 8)` at the top of the file to match.
The animation speed is set by `time.sleep(0.01)` in `Astar_vis`.

## File overview

| Function                   | Purpose                                                   |
|----------------------------|-----------------------------------------------------------|
| `Node`                     | Search state: position, cost, heading, parent             |
| `Node.get_neighbors_state` | Forward / turn-right / turn-left successors               |
| `Node.absolute_distance`   | Euclidean heuristic to the goal                           |
| `Astar_vis`                | Main A* loop with live visualization                      |
| `reconstruct_path`         | Walks parent links to build the path                      |
| `reconstruct_path_visual`  | Clears the search colors and draws the final path         |
| `gen_map_img`              | Converts the grid values to RGB colors                    |
| `draw_pointer`             | Draws a heading arrow on a cell                           |
| `idx_pos_in_list`          | Finds a `(position, direction)` state in the open/closed list |

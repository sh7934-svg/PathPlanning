import matplotlib.pyplot as plt
import numpy as np
import time

bounds = (10, 8)

class Node():
    def __init__(self, pos, g, goal_pos, parent, direct):
        self.pos = pos
        self.g = g
        self.f = g + self.absolute_distance(goal_pos)
        self.parent = parent
        self.direct = direct

        #0 up
        #1 right
        #2 down
        #3 left

    def get_neighbors_state(self, map):
        #1 move forward
        #2 turn right
        #3 turn left

        neighbor = []

        #Forward

        if self.direct == 0:
            new_position = (self.pos[0] - 1, self.pos[1])
        elif self.direct == 1:
            new_position = (self.pos[0], self.pos[1] + 1)
        elif self.direct == 2:
            new_position = (self.pos[0] + 1, self.pos[1])
        elif self.direct == 3:
            new_position = (self.pos[0], self.pos[1] - 1)
        
        y = new_position[0]
        x = new_position[1]
        
        if (0 <= y < bounds[0]) and (0 <= x < bounds[1]):
            if (map[new_position] != 1):
                neighbor.append((new_position, self.direct, 1))
        
        #Right
        neighbor.append((self.pos, (self.direct + 1)%4, 1))

        #Left
        neighbor.append((self.pos, (self.direct - 1)%4, 1))

        return neighbor

    
    def absolute_distance(self, goal_pos):
        dx = self.pos[0] - goal_pos[0]
        dy = self.pos[1] - goal_pos[1]
        total_distance = np.sqrt(np.square(dx) + np.square(dy))
        
        return total_distance

def gen_map_img(map):
# 1 corresponds to obstacle so black
# 2 corresponds to end so red
# 3 corresponds to start so green
# 4 corresponds to open so grey
# 5 corresponds to closed so blue
# 6 corresponds to actual path so yellow

    rows = map.shape[0]
    cols = map.shape[1]

    map_img = np.ones((rows, cols, 3))
    for i in range(rows):
        for j in range(cols):
            if map[i,j] == 1:
                map_img[i,j] = [0, 0, 0]
            elif map[i, j] == 2:
                map_img[i,j] = [1, 0, 0]
            elif map[i, j] == 3:
                map_img[i,j] = [0, 1, 0]
            elif map[i, j] == 4:
                map_img[i,j] = [0.5, 0.5, 0.5]
            elif map[i, j] == 5:
                map_img[i,j] = [0, 0, 1]
            elif map[i, j] == 6:
                map_img[i,j] = [1, 1, 0]
            elif map[i, j] == 7:
                map_img[i,j] = [1, 1, 1]

    return map_img

def find_lowest_index_f(nodes):
    lowest_index = 0

    for i in range(1, len(nodes)):
        if nodes[i].f < nodes[lowest_index].f:
            lowest_index = i
    
    return lowest_index

def idx_pos_in_list(curr_pos, direction, nodes):
    for i in range(len(nodes)):
        if nodes[i].pos == curr_pos and nodes[i].direct == direction:
            return i
        
    return -1

def reconstruct_path(node, closed_list):
    path = [(node.pos, node.direct)]
    parent = node.parent
    print(type(parent))

    while parent is not None:
        path.append((parent.pos, parent.direct))
        parent = parent.parent

    path.reverse()
    return path

def draw_pointer(ax, pos, direction):
    arrows = {
        0: "↑",
        1: "→",
        2: "↓",
        3: "←",
    }

    ax.text(pos[1], pos[0], arrows[direction], color="black", ha="center", va="center", fontsize=18, fontweight="bold")

def reconstruct_path_visual(y_bound, x_bound, goal, start, path, map):
        for h in range(y_bound):
            for j in range(x_bound):
                if map[h,j] == 4 or map[h,j] == 5:
                    map[h,j] = 7
        
        for i in range(len(path)):
            curr_node_pos = path[i][0]
            curr_node_direct = path[i][1]
            map[start] = 3
            map[curr_node_pos] = 6
            map[goal] = 2
            draw_pointer(ax, curr_node_pos, curr_node_direct)
            img.set_data(gen_map_img(map))
            fig.canvas.draw()          # Redraw the canvas
            fig.canvas.flush_events()

def Astar_vis(map, start, goal, img, fig, ax):
    fig.canvas.draw()          # Redraw the canvas
    fig.canvas.flush_events()
    #tuples in from ((x, y), g, h)
    open_list = [Node(start, 0, goal, None, 0)]
    closed_list = []
    while open_list:
        time.sleep(0.01)

        current_index = find_lowest_index_f(open_list)
        current_node = open_list[current_index]
        current_pos = open_list[current_index].pos

        # Reached Goal
        if current_pos == goal:
            total_cost = current_node.f
            path = reconstruct_path(current_node, closed_list)
            reconstruct_path_visual(bounds[0], bounds[1], goal, start, path, map)
            return f"Found Path. Total cost was {total_cost}"
        
        if current_pos is not start and current_pos is not goal:
            map[current_pos] = 5

        closed_list.append(open_list.pop(current_index))

        neighbors = current_node.get_neighbors_state(map)

        for i in range(len(neighbors)):
            curr_neighbor_pos = neighbors[i][0]
            curr_neighbor_direct = neighbors[i][1]
            curr_neighbor_cost = neighbors[i][2]
            curr_neighbor_idx_closed = idx_pos_in_list(curr_neighbor_pos, curr_neighbor_direct, closed_list)
            curr_neighbor_idx_open = idx_pos_in_list(curr_neighbor_pos, curr_neighbor_direct, open_list)

            if curr_neighbor_idx_closed >= 0:
                continue

            tent_g = current_node.g + curr_neighbor_cost

            if curr_neighbor_idx_open < 0:
                open_list.append(Node(curr_neighbor_pos, tent_g, goal, current_node, curr_neighbor_direct))
                if curr_neighbor_pos != goal:
                    map[curr_neighbor_pos] = 4
            elif tent_g >= open_list[curr_neighbor_idx_open].g:
                continue

            open_list[curr_neighbor_idx_open].g = tent_g
            open_list[curr_neighbor_idx_open].f = tent_g + open_list[curr_neighbor_idx_open].absolute_distance(goal)
            open_list[curr_neighbor_idx_open].parent = current_node
            open_list[curr_neighbor_idx_open].direct = curr_neighbor_direct

        #visualization
        img.set_data(gen_map_img(map))

        fig.canvas.draw()          # Redraw the canvas
        fig.canvas.flush_events()
    return "Failed to find path"


target = (0, 7)
start = (9, 0)

obstacles = [
    (0,0),
    (1,1), (1,3), (1,4), (1,5),
    (2,1), (2,5),
    (3,1), (3,3), (3,5),
    (4,3),
    (5,1), (5,3), (5,5),
    (6,1), (6,5),
    (7,3),
    (8,3), (8,4),
    (9,6), (9,7)
    ]

map = np.zeros((10, 8))

for i, j in obstacles:
    map[i, j] = 1

map[target] = 2
map[start] = 3

plt.ion()

fig, ax = plt.subplots()
img = ax.imshow(gen_map_img(map)) # vmin/vmax for colormap scaling

print(Astar_vis(map, start, target, img, fig, ax))

plt.ioff() # Turn off interactive mode at the end
plt.show() # Keep the final plot open (optional)
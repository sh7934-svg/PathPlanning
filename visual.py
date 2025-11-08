import matplotlib.pyplot as plt
import numpy as np
import time

bounds = (10, 8)

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
    

def get_neighbors(pos, map):
    neighbors = [
        (pos[0] - 1, pos[1]), (pos[0] + 1, pos[1]),
        (pos[0], pos[1] - 1), (pos[0], pos[1] + 1), 
        (pos[0] - 1, pos[1] - 1), (pos[0] + 1, pos[1] + 1), 
        (pos[0] - 1, pos[1] + 1), (pos[0] + 1, pos[1] - 1), 
    ]

    idx = 0

    while idx < len(neighbors):
        current_neighbors = neighbors[idx]
        if 0 > current_neighbors[0] or bounds[0] <= current_neighbors[0] or 0 > current_neighbors[1] or bounds[1] <= current_neighbors[1]:
            neighbors.pop(idx)
        elif map[current_neighbors] == 1:
            neighbors.pop(idx)
        else:
            idx += 1

    return neighbors

def absolute_distance(curr_pos, goal_pos):
    dx = curr_pos[0] - goal_pos[0]
    dy = curr_pos[1] - goal_pos[1]
    total_distance = np.sqrt(np.square(dx) + np.square(dy))
    
    return total_distance

def find_lowest_index_f(nodes):
    lowest_index = 0

    for i in range(1, len(nodes)):
        if nodes[i][2] < nodes[lowest_index][2]:
            lowest_index = i
    
    return lowest_index

def idx_pos_in_list(curr_pos, nodes):
    for i in range(len(nodes)):
        if nodes[i][0] == curr_pos:
            return i
        
    return -1

def reconstruct_path(node, closed_list):
    path = [node[0]]
    parent = node[3]

    while parent is not None:
        for n in closed_list:
            if n[0] == parent:
                path.append(n[0])
                parent = n[3]
                break
    path.reverse()
    return path

def Astar_vis(map, start, goal, img, fig):
    fig.canvas.draw()          # Redraw the canvas
    fig.canvas.flush_events()

    #tuples in from ((x, y), g, h)
    open_list = [[start, 0, absolute_distance(start, goal), None]]
    closed_list = []
    while open_list:
        time.sleep(0.1)

        current_index = find_lowest_index_f(open_list)
        current_node = open_list[current_index]
        current_pos = open_list[current_index][0]

        if current_pos == goal:
            path = reconstruct_path(current_node, closed_list)
            for i in path:
                map[i] = 6
            for h in range(bounds[0]):
                for j in range(bounds[1]):
                    if map[h,j] == 4 or map[h,j] == 5:
                        map[h,j] = 7
            map[start] = 3
            map[goal] = 2
            img.set_data(gen_map_img(map))
            fig.canvas.draw()          # Redraw the canvas
            fig.canvas.flush_events()
            return "Found Path"
        
        if current_pos is not start and current_pos is not goal:
            map[current_pos] = 5

        closed_list.append(open_list.pop(current_index))

        neighbors = get_neighbors(current_pos, map)

        for i in range(len(neighbors)):
            curr_neighbor = neighbors[i]

            if idx_pos_in_list(curr_neighbor, closed_list) >= 0:
                continue

            tent_g = current_node[1] + absolute_distance(current_pos, curr_neighbor)

            curr_neighbor_idx = idx_pos_in_list(curr_neighbor, open_list)
            if curr_neighbor_idx < 0:
                open_list.append([curr_neighbor, tent_g, tent_g+absolute_distance(curr_neighbor, goal), current_pos])
                if curr_neighbor != goal:
                    map[curr_neighbor] = 4
            elif tent_g >= open_list[curr_neighbor_idx][1]:
                continue

            open_list[curr_neighbor_idx][1] = tent_g
            open_list[curr_neighbor_idx][2] = tent_g + absolute_distance(curr_neighbor, goal)
            open_list[curr_neighbor_idx][3] = current_pos

        #visualization
        img.set_data(gen_map_img(map))

        fig.canvas.draw()          # Redraw the canvas
        fig.canvas.flush_events()
    return "Failed to find path"


target = (0, 7)
start = (9, 0)

obstacles = [(1, 3), (2, 0), (2, 1), (2, 3), (3,1), (3, 3), (4, 1), (4, 3), (5, 1), (6, 3)]

map = np.zeros((10, 8))

for i, j in obstacles:
    map[i, j] = 1

map[target] = 2
map[start] = 3

plt.ion()

fig, ax = plt.subplots()
img = ax.imshow(gen_map_img(map)) # vmin/vmax for colormap scaling

print(Astar_vis(map, start, target, img, fig))

plt.ioff() # Turn off interactive mode at the end
plt.show() # Keep the final plot open (optional)
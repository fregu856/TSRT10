# NOTE! This is just a copy of balrog/python_scripts/utilities.py

# This code contains helper function that all concern transforms from/to
# (row, col) indices in map matrices to/from real world positions [x, y]. The
# file is imported in main.py and nav_covering.py.

import numpy as np

# function for transforming a (row, col) map matrix index into its corresponding
# real world position [x, y]:
def map_index_2_pos(map_origin_obj, map_resolution, pos_index):
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    x_map_ind = pos_index[0]
    y_map_ind = pos_index[1]

    x_map = x_map_ind*map_resolution + map_resolution/2
    y_map = y_map_ind*map_resolution + map_resolution/2

    x = x_map + map_origin[0]
    y = y_map + map_origin[1]

    pos = [x, y]

    return pos

# function for transforming a real world position [x, y] into its corresponding
# (row, col) map matrix index:
def pos_2_map_index(map_origin_obj, map_resolution, pos):
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    x = pos[0]
    y = pos[1]

    x_map = x - map_origin[0]
    y_map = y - map_origin[1]

    x_map_ind = int(x_map/map_resolution) # (col)
    y_map_ind = int(y_map/map_resolution) # (row)

    pos_index = [x_map_ind, y_map_ind]

    return pos_index

# function for transforming a raw path (a numpy array containing all row indices
# and a numpy array containing all col indices) into a path of real world
# positions [x, y] (output format: [x1, y1, x2, y2, ..., xn, yn]):
def raw_path_2_path(raw_path, map_origin_obj, map_resolution):
    rows = raw_path[0].tolist()
    cols = raw_path[1].tolist()
    path = []

    for (row, col) in zip(rows, cols):
        pos_index = [col, row]
        pos = map_index_2_pos(map_origin_obj, map_resolution, pos_index)
        path += [pos[0]]
        path += [pos[1]]

    return path

# function for transforming an OccupancyGrid map message into a matrix (2D
# numpy array):
def map_msg_2_matrix(map_msg):
    map_data = map_msg.data
    map_height = map_msg.info.height

    # convert the map data from a list into a numpy array:
    map_matrix = np.array(map_data)
    # split the array into a list of map_height arrays (each list element is a row):
    map_matrix = np.split(map_matrix, map_height)
    # convert the list of arrays into a 2D array:
    map_matrix = np.array(map_matrix)

    return map_matrix

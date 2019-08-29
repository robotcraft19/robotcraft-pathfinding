from __future__ import print_function
import numpy as np
import os

complex_a = True

class Node:
    target = None
    directions = None
    matrix = None

    def __init__(self, row, column, g, parent):
        self.row = row
        self.column = column
        self.parent = parent
        self.g = g # total cost so far
        self.h = self.calculate_heuristic() # estimated cost from node to goal
        self.f = self.g + self.h + self.calculate_wall_bias()# total estimated cost of path through node

    def calculate_heuristic(self):
        if Node.target == None:
            return 0

        global complex_a
        if complex_a == False: # only for directions
            # return manhattan distance
            return abs(self.row - Node.target.row) + abs(self.column - Node.target.column)
        else:
            # return euclidian distance
            return ((self.row - Node.target.row)**2 + (self.column - Node.target.column)**2)**0.5

    def calculate_wall_bias(self):
        wall_penalty = 0
        for dir in Node.directions:
            try:
                nb =  Node.matrix[self.row + dir[0]][self.column + dir[1]]
                if nb == 1:
                    wall_penalty += 10.0
                for dir_dir in Node.directions:
                    try:
                        nb_nb =  Node.matrix[self.row + dir[0] + dir_dir[0]][self.column + dir[1] + dir_dir[1]]
                        if nb_nb == 1:
                            wall_penalty += 5.0
                    except:
                        # out of bounds
                        pass
            except:
                # out of bounds
                pass

        return wall_penalty

    def __lt__(self, other):
        # comparison method for sorting priority
        return self.f < other.f

    def __eq__(self, other):
        # check if nodes have same coordinates
        if (isinstance(other, Node)):
            return self.row == other.row and self.column == other.column
        return False

    def __str__(self):
        return 'Node({}, {})'.format(self.row, self.column)

    def __repr__(self):
        return 'Node({}, {}, {}, {})'.format(self.row, self.column, self.g, self.parent)


class PathFinder:

    def __init__(self, matrix):
        self.matrix = np.array(matrix)
        Node.matrix = self.matrix
        # Initialize matrix with possible movements and their cost
        self.directions = self.initialize_directions()

        # Get matrix coorinates of target position
        result = np.where(self.matrix == -2)
        self.target = Node(result[0][0], result[1][0], -1, None) # extract indices

        # Make target position globally accessible within Node class for distance calculation
        Node.target = self.target

        # Get matrix coordinates of initial robot position and create starting node
        result = np.where(self.matrix == -1)
        self.start = Node(result[0][0], result[1][0], 0, None) # extract indices

        # Initialize open nodes queue containing start node
        self.open_nodes = [self.start]
        # Initialize empty closed nodes queue
        self.closed_nodes = []

        # Start A* algorithm to calculate paths
        self.calculate_path()

    def calculate_path(self):
        while(len(self.open_nodes) > 0):
            self.open_nodes.sort() # sort list according to their f values
            current_node = self.open_nodes[0] # extract node with highest priority
            if current_node == self.target:
                path = self.reconstruct_path(current_node)
                return path

            else:
                # Remove current node from open nodes list and append to closed list
                self.closed_nodes.append(self.open_nodes.pop(0))

                # Create list of neighbor nodes
                neighbors = [Node(current_node.row + dir[0], current_node.column +dir[1],
                    current_node.g + dir[2], current_node) for dir in self.directions]
                # Filter nodes that are out of bounds
                neighbors = [nb for nb in neighbors
                    if 0 <= nb.row < self.matrix.shape[0]
                        and 0 <= nb.column < self.matrix.shape[1]]
                # Filter nodes that are occupied
                neighbors = [nb for nb in neighbors if self.matrix[nb.row][nb.column] != 1]

                for neighbor_node in neighbors:
                    # Check if neighbor_node is in open nodes list
                    if neighbor_node in self.open_nodes:
                        # Extract pre-existing neighbor node that has same coordinates
                        # but different g and f values
                        ex_nb = self.open_nodes.pop(self.open_nodes.index(neighbor_node))

                        # Add current path to neighbor to list ist better
                        if neighbor_node.g < ex_nb.g:
                            self.open_nodes.append(neighbor_node)
                        else:
                            #Otherwise readd existing path to neighbor
                            self.open_nodes.append(ex_nb)
                    elif not neighbor_node in self.closed_nodes:
                        # Otherwise add neighbor to open nodes list
                        self.open_nodes.append(neighbor_node)

        # No path found, ran out of open nodes
        print("============NO PATH TO TARGET FOUND============")
        return [(self.start.row, self.start.column)] # no movement

    def reconstruct_path(self, node):
        current_node = node
        path = []
        while(current_node != self.start):
            path.append(current_node)
            current_node = current_node.parent

        path.reverse()
        path = [(node.row, node.column) for node in path]
        self.write_map(path)
        return path

    def write_map(self, path, print_output=False):
        # Writes map to file and optionally prints it
        map = self.matrix.copy()
        with open(os.path.join(os.path.expanduser('~'),
            'catkin_ws/src/robotcraft_maze/scans/path_route.txt'), 'w') as f:
            for point in path:
                map[point[0], point[1]] = 7
            for row in map:
                for col in row:
                    f.write(str(int(col)))
                    if print_output == True:
                        print(col, end = '')
                f.write('\n')
                if print_output == True:
                    print()

    def initialize_directions(self):
        global complex_a
        directions = []
        if complex_a  == True:
            # matrix including row and column translation and movement cost
            directions = [
                [1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [1, 1, 2**0.5],
                [-1, 1, 2**0.5],
                [1, -1, 2**0.5],
                [-1, -1, 2**0.5],
            ]
        else:
            # matrix including row and column translation and movement cost
            directions = [
                [1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1]
            ]
        Node.directions = directions
        return directions

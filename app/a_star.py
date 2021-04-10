import json
import math

class Node():

    """" A class that defines a node and its attributes. """

    def __init__(self, state, parent, action, coord, neighbors, cost):
        self.state = state # State is the name (e.g. "Library")
        self.parent = parent # Previous node
        self.action = action # Description of how we got from the parent node to this one
        self.coord = coord # Coordinates of the node
        self.neighbors = neighbors # Neighbor nodes
        self.cost = cost # Distance between this node and the parent node

class Frontier():

    """ A class that defines the frontier and its associated methods. The frontier is basically
    a list that keeps track of the nodes that haven't been explored yet. Though, the frontier 
    doesn't start with all the nodes in it. Only the neighbors of explored nodes get added to the
    frontier. """

    def __init__(self):
        self.frontier = [] # Create an empty list at the beginning
    
    def add(self, node):
        self.frontier.append(node) # Append a node (used when there are new neighbors)
    
    def contains_state(self, state):
        return any(node.state == state for node in self.frontier) # Check if a node is already in the frontier
    
    def empty(self):
        return len(self.frontier) == 0 # Check if the frontier is empty
    
    def calculate_distance(self, coord_a, coord_b):
        
        # Calculate the distance between 2 points
        distance = math.sqrt((coord_b[0]-coord_a[0])**2 + (coord_b[1]-coord_a[1])**2) 
        return distance

    def remove(self, coord_end):
        
        # Pick the next node to explore and remove it from the frontier
        if self.empty():
            raise Exception("empty frontier")
        else:
            # Uncomment below for depth first search (and comment heuristics method)
            # closest_node = self.frontier[-1]
            # self.frontier = self.frontier[:-1]

            # Compute heuristics (i.e. distance from goal + current path cost)
            min_cost = math.inf
            for node in self.frontier:

                # Calculate the cost for each node and pick the lowest
                cost = node.cost + self.calculate_distance(node.coord, coord_end)
                if cost < min_cost:
                    min_cost = cost
                    closest_node = node
            self.frontier.remove(closest_node)
            return closest_node

class A_Star():

    """ The class that finds the quickest way from a start node to an end node using the core of this class: the solve()
    method. Apart from the format_output() method, which turns the output into human language, all other methods are just helping
    methods. """

    def __init__(self, filename, start, end):

        # Load JSON
        with open(filename, 'r') as f:
            self.campus = json.load(f)

        # Initialize values for the first node (and end node state)
        self.start = start
        self.end = end
        self.starting_actions = self.campus[start]["actions"]
        self.starting_coords = self.campus[start]["coords"]
        self.starting_neighbors = self.campus[start]["neighbors"]
    
    def neighbors(self, node):
        
        # Find all the neighbors of a node using the data loaded from the JSON
        result = []
        for i in range(len(node.neighbors)):
            result.append((self.campus[node.state]["actions"][i], node.neighbors[i], self.campus[node.neighbors[i]]["coords"], self.campus[node.neighbors[i]]["neighbors"]))
        return result
    
    def calculate_distance(self, coord_a, coord_b):
        
        # Calculate the distance between 2 points
        distance = math.sqrt((coord_b[0]-coord_a[0])**2 + (coord_b[1]-coord_a[1])**2)
        return distance

    def print_solution(self):
        solution = self.solution
        print(solution)

    def solve(self):

        # Initialize frontier with starting point
        start_node = Node(state=self.start, parent=None, action=self.starting_actions, coord=self.starting_coords, neighbors=self.starting_neighbors, cost=0)
        frontier = Frontier()
        frontier.add(start_node)

        # Initialize an empty explored set
        self.explored = set()

        # Keep looking until solution found
        while True:

            # If nothing left in frontier, then no path
            if frontier.empty():
                raise Exception("no solution")
            
            # Pick a node
            node = frontier.remove(self.campus[self.end]["coords"])

            # If the node is the goal, then solution found
            if node.state == self.end:
                nodes = []

                # Go backwards to collect the full path using the parent argument of each node
                while node.parent is not None:
                    nodes.append(node)
                    node = node.parent
                nodes.append(node) # add start node
                nodes.reverse() # put it in the right order
                self.solution = nodes
                return
            
            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state, coord, neighbors in self.neighbors(node):

                # Check that this not is not already in the frontier or hasn't already been explored
                if not frontier.contains_state(state) and state not in self.explored:
                    
                    # Increase cost
                    cost = node.cost + self.calculate_distance(node.coord, coord)

                    # Create node and add it to the frontier
                    child = Node(state=state, parent=node, action=action, coord=coord, neighbors=neighbors, cost=cost)
                    frontier.add(child)

    def calculate_directions():

        # Initialize dict
        directions = {}

        # Loop over all groups of 3 consecutive nodes
        for i in range(len(self.solution)-2):

            # Calculate distances between points
            a = self.calculate_distance(self.solution[i].coord, self.solution[i+1].coord)
            b = self.calculate_distance(self.solution[i+1].coord, self.solution[i+2].coord)
            c = self.calculate_distance(self.solution[i+2].coord, self.solution[i].coord)

            # Calculate the angle between a and b
            angle = math.acos((a**2 + b**2 - c**2)/(2*a*b))

            # Check if the next node is ahead
            if angle > (5*math.pi/6):
                direction = "straight"
            
            # If there is a right or left turn
            elif angle > (math.pi/6):
                
                # To determine if it is left or right, move all points so that the first point is at coordinates (0,0)
                a_coord = [self.solution[i].coord[0]-self.solution[i].coord[0], self.solution[i].coord[1]-self.solution[i].coord[1]]
                b_coord = [self.solution[i+1].coord[0]-self.solution[i].coord[0], self.solution[i+1].coord[1]-self.solution[i].coord[1]]
                c_coord = [self.solution[i+2].coord[0]-self.solution[i].coord[0], self.solution[i+2].coord[1]-self.solution[i].coord[1]]
                
                # Now check all possible cases
                # If x coordinate of b is positive: if c_coord is above [AB] -> left, else -> right
                if b_coord[0] > 0:

                    # Calculate the slope of the line that goes from a_coord to b_coord
                    slope = b_coord[1]/b_coord[0]

                    if c_coord[1] > (c_coord[0]*slope):
                        direction = "left"
                    else:
                        direction = "right"
                
                # If x coord of b is negative
                elif b_coord[0] < 0:

                    # Calculate the slope of the line that goes from a_coord to b_coord
                    slope = b_coord[1]/b_coord[0]

                    if c_coord[1] > (c_coord[0]*slope):
                        direction = "right"
                    else:
                        direction = "left"
                
                # If x coord of b = 0
                else:
                    # If y > 0
                    if b_coord[1] > 0:
                        if c_coord[0] > 0:
                            direction = "right"
                        else:
                            direction = "left"
                    # If y < 0
                    else:
                        if c_coord[0] > 0:
                            direction = "left"
                        else:
                            direction = "right"

            else:
                direction = "backwards"
            
            # Add direction to the dict
            directions[self.solution[i+1]] = direction
        
        return directions

    def format_output():

        # Initialize instructions
        instructions = []
        
        # Calculate the angles of middle nodes
        directions = self.calculate_directions()

        # Loop over all nodes and compute instructions
        for i in range(len(self.solution)):

            # Start node
            if i == 0:

                # If the starting point is within a building
                if "building" in self.solution[i].tags:
                    instructions.append["Right, start by leaving this building through the main door"]
            

        



if __name__ == "__main__":

    start = "out_a"
    end = "build_e"
    campus = A_Star("test.json", start, end)
    campus.solve()
    campus.print_solution()
    print(campus.solution)
    for node in campus.solution:
        print(node.state)
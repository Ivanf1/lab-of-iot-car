import sys

class PathFinder:
    graph = None

    def __init__(self) -> None:
        self.graph = Graph()

    def compute_path(self, stops):
        path = self.graph.compute_path(stops=stops)
        directions = self.graph.compute_directions_for_path(path=path)

        return path, directions
    
 
class Graph():
    nodes = ["Start", "Start_intersection_from_P0", "Start_intersection_from_P1", "P1_intersection", "P0_intersection", "P1C0", "P1C1", "P0C0", "P0C1"]
    graph = {}
    direction_graph = {}

    def __init__(self):
        self.init_graph()
        self.init_directions_graph()

    def init_graph(self):
        for node in self.nodes:
            self.graph[node] = {}

        self.graph["Start"]["Start_intersection_from_P0"] = 1
        self.graph["Start"]["Start_intersection_from_P1"] = 1

        self.graph["Start_intersection_from_P0"]["P1_intersection"] = 1
        self.graph["P1_intersection"]["Start_intersection_from_P0"] = 20

        self.graph["Start_intersection_from_P0"]["P0_intersection"] = 1
        self.graph["P0_intersection"]["Start_intersection_from_P0"] = 1

        self.graph["Start_intersection_from_P1"]["P0_intersection"] = 1
        self.graph["P0_intersection"]["Start_intersection_from_P1"] = 20

        self.graph["Start_intersection_from_P1"]["P1_intersection"] = 1
        self.graph["P1_intersection"]["Start_intersection_from_P1"] = 1

        self.graph["P0_intersection"]["P0C0"] = 1
        self.graph["P1_intersection"]["P1C0"] = 1
        self.graph["P1C0"]["P1C1"] = 1
        self.graph["P0C0"]["P0C1"] = 1

        self.graph = self.build_complete_graph()
    
    def init_directions_graph(self):
        for node in self.nodes:
            self.direction_graph[node] = {}

        # directions
        self.direction_graph["P0C0"]["P0C1"] = "straight"
        self.direction_graph["P0_intersection"]["P0C0"] = "r"
        self.direction_graph["Start_intersection_from_P0"]["P0_intersection"] = "l"
        self.direction_graph["Start_intersection_from_P0"]["P1_intersection"] = "straight"
        self.direction_graph["Start"]["Start_intersection_from_P0"] = "straight"
        # ---------------------------------------------------------------------------
        self.direction_graph["Start"]["Start_intersection_from_P1"] = "straight"
        self.direction_graph["Start_intersection_from_P1"]["P1_intersection"] = "r"
        self.direction_graph["Start_intersection_from_P1"]["P0_intersection"] = "straight"
        self.direction_graph["P1_intersection"]["P1C0"] = "l"
        self.direction_graph["P1C0"]["P1C1"] = "straight"

        # reversed directions
        self.direction_graph["P0C1"]["P0C0"] = "straight"
        self.direction_graph["P0C0"]["P0_intersection"] = "straight"
        self.direction_graph["P0_intersection"]["Start_intersection_from_P0"] = "l"
        self.direction_graph["Start_intersection_from_P0"]["Start"] = "r"
        # ---------------------------------------------------------------------------
        self.direction_graph["Start_intersection_from_P1"]["Start"] = "l"
        self.direction_graph["P1_intersection"]["Start_intersection_from_P1"] = "r"
        self.direction_graph["P1C0"]["P1_intersection"] = "straight"
        self.direction_graph["P1C1"]["P1C0"] = "straight"
        
    def build_complete_graph(self):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from
         node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in self.nodes:
            graph[node] = {}
        
        graph.update(self.graph)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    if adjacent_node == "Start_intersection_from_P0" or adjacent_node == "Start_intersection_from_P1":
                        if node == "P1_intersection" or node == "P0_intersection":
                            continue
                    graph[adjacent_node][node] = value
        
        return graph
    
    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes
    
    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

    def dijkstra_algorithm(self, start_node):
        unvisited_nodes = list(self.get_nodes())

        # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
        shortest_path = {}

        # We'll use this dict to save the shortest known path to a node found so far
        previous_nodes = {}

        # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
        max_value = sys.maxsize
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        # However, we initialize the starting node's value with 0   
        shortest_path[start_node] = 0
        
        # The algorithm executes until we visit all nodes
        while unvisited_nodes:
            # The code block below finds the node with the lowest score
            current_min_node = None
            for node in unvisited_nodes: # Iterate over the nodes
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
                    
            # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = self.get_outgoing_edges(current_min_node)
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + self.value(current_min_node, neighbor)
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node

            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        
        return previous_nodes, shortest_path

    def compute_path(self, stops):
        paths = []

        for idx, stop in enumerate(stops):
            if (idx == 0):
                start_node = "Start"
            else:
                start_node = stops[idx - 1]
            previous_nodes, _ = self.dijkstra_algorithm(start_node=start_node)
            paths.append(self.compute_path_piece(previous_nodes=previous_nodes, start_node=start_node, target_node=stop))

        # go back to Start from the last stop
        previous_nodes, _ = self.dijkstra_algorithm(start_node=stops[-1])
        paths.append(self.compute_path_piece(previous_nodes=previous_nodes, start_node=stops[-1], target_node="Start"))

        return self.connect_paths(paths=paths)

    def compute_path_piece(self, previous_nodes, start_node, target_node):
        path = []
        node = target_node
        
        while node != start_node:
            path.append(node)
            node = previous_nodes[node]
    
        # Add the start node manually
        path.append(start_node)
        list.reverse(path)
        return path

    def connect_paths(self, paths):
        paths_joined = []
        for idx, path in enumerate(paths):
            if (idx == 0):
                for stop in path:
                    paths_joined.append(stop)
                continue

            path.pop(0)
            for stop in path:
                paths_joined.append(stop)
        
        # print(paths_joined)
        # for idx, stop in enumerate(paths_joined):
        #     print(str(idx) + " - " + stop)
        return paths_joined
    
    def compute_directions_for_path(self, path):
        directions = []

        for idx, stop in enumerate(path):
            if idx == len(path) - 1:
                continue
            # print(stop + " - " + path[idx + 1])
            directions.append(self.direction_graph[stop][path[idx + 1]])

        # print('\n')
        # for idx, stop in enumerate(directions):
        #     print(str(idx) + " - " + stop)
        
        return directions
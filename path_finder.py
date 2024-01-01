import requests
import sys

class PathFinder:
    url = "http://localhost:3000/api/cube"

    def __init__(self) -> None:
        pass

    def get_cube_positions_for_person(self):
        response = requests.get(self.url)
        response_json = response.json()
        print(response_json)
 
class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.build_complete_graph(nodes, init_graph)
        
    def build_complete_graph(self, nodes, init_graph):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from
         node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}
        
        graph.update(init_graph)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
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

def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())

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
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node

        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(current_min_node)
    
    return previous_nodes, shortest_path

def print_result(previous_nodes, start_node, target_node):
    path = []
    node = target_node
    
    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
 
    # Add the start node manually
    path.append(start_node)
    # path = reversed(path)
    list.reverse(path)

    out = []
    i = 0
    while i < len(path) -1:
        out.append(path[i] + " (" + direction_graph[path[i]][path[i+1]]+ ")")
        i += 1
    out.append(path[len(path)-1])

    print(" -> ".join(out))

def compute_path(stops):
    for idx, stop in enumerate(stops):
        if (idx == 0):
            start_node = "Start"
        else:
            start_node = stops[idx - 1]
        previous_nodes, _ = dijkstra_algorithm(graph=graph, start_node=start_node)
        print_result(previous_nodes, start_node=start_node, target_node=stop)

    # go back to Start from the last stop
    previous_nodes, _ = dijkstra_algorithm(graph=graph, start_node=stops[-1])
    print_result(previous_nodes, start_node=stops[-1], target_node="Start")

nodes = ["Start","Start_intersection_from_P0","Start_intersection_from_P1", "P1_intersection", "P0_intersection", "P1C0", "P1C1", "P0C0", "P0C1"]
 
init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph["Start"]["Start_intersection_from_P0"] = 1
init_graph["Start"]["Start_intersection_from_P1"] = 1
init_graph["Start_intersection_from_P0"]["Start_intersection_from_P1"] = 2
init_graph["Start_intersection_from_P0"]["P0_intersection"] = 1
init_graph["Start_intersection_from_P1"]["P1_intersection"] = 1
init_graph["P0_intersection"]["P0C0"] = 1
init_graph["P1_intersection"]["P1C0"] = 1
init_graph["P1C0"]["P1C1"] = 1
init_graph["P0C0"]["P0C1"] = 1

direction_graph = {}
for node in nodes:
    direction_graph[node] = {}

# directions
direction_graph["P0C0"]["P0C1"] = "straight"
direction_graph["P0_intersection"]["P0C0"] = "r"
direction_graph["Start_intersection_from_P0"]["P0_intersection"] = "l"
direction_graph["Start"]["Start_intersection_from_P0"] = "straight"
# ---------------------------------------------------------------------------
direction_graph["Start"]["Start_intersection_from_P1"] = "straight"
direction_graph["Start_intersection_from_P1"]["P1_intersection"] = "r"
direction_graph["P1_intersection"]["P1C0"] = "l"
direction_graph["P1C0"]["P1C1"] = "straight"


# reversed directions
direction_graph["P0C1"]["P0C0"] = "straight"
direction_graph["P0C0"]["P0_intersection"] = "straight"
direction_graph["P0_intersection"]["Start_intersection_from_P0"] = "l"
direction_graph["Start_intersection_from_P0"]["Start"] = "straight"
# ---------------------------------------------------------------------------
direction_graph["Start_intersection_from_P1"]["Start"] = "straight"
direction_graph["P1_intersection"]["Start_intersection_from_P1"] = "r"
direction_graph["P1C0"]["P1_intersection"] = "straight"
direction_graph["P1C1"]["P1C0"] = "straight"

graph = Graph(nodes, init_graph)
# compute_path(["P1C0", "P1C1", "P0C0"]) # error!
compute_path(["P0C0"])
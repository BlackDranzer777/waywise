import arcpy
import heapq

def dijkstra(graph, start_node, end_node):
    """
    Implementation of Dijkstra's algorithm to find the shortest path.
    
    Parameters:
    graph: dict
        A dictionary representing the graph where keys are node IDs and values are lists of tuples (neighbor_node, weight).
    start_node: any hashable type
        The starting node ID.
    end_node: any hashable type
        The destination node ID.
    
    Returns:
    path: list
        The shortest path from start_node to end_node.
    distance: float
        The total distance of the shortest path.
    """
    priority_queue = []
    heapq.heappush(priority_queue, (0, start_node))
    distances = {node: float('inf') for node in graph}
    distances[start_node] = 0
    previous_nodes = {node: None for node in graph}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_node == end_node:
            path = []
            while previous_nodes[current_node] is not None:
                path.insert(0, current_node)
                current_node = previous_nodes[current_node]
            path.insert(0, start_node)
            return path, distances[end_node]
        
        if current_distance > distances[current_node]:
            continue
        
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return None, float('inf')

def build_graph_from_arcpy(edges_fc):
    """
    Builds a graph from an ArcPy feature class representing edges.
    
    Parameters:
    edges_fc: str
        The path to the feature class containing edges.
    
    Returns:
    graph: dict
        A dictionary representing the graph where keys are node IDs and values are lists of tuples (neighbor_node, weight).
    """
    graph = {}
    with arcpy.da.SearchCursor(edges_fc, ['StartNode', 'EndNode', 'Weight']) as cursor:
        for row in cursor:
            start_node, end_node, weight = row
            if start_node not in graph:
                graph[start_node] = []
            if end_node not in graph:
                graph[end_node] = []
            graph[start_node].append((end_node, weight))
            graph[end_node].append((start_node, weight))  # If the graph is undirected
    
    return graph

def main():
    arcpy.env.workspace = "path_to_your_geodatabase"
    edges_fc = "path_to_your_edge_feature_class"
    
    graph = build_graph_from_arcpy(edges_fc)
    
    start_node = "start_node_id"
    end_node = "end_node_id"
    
    path, distance = dijkstra(graph, start_node, end_node)
    
    if path is not None:
        print(f"Shortest path from {start_node} to {end_node} is: {' -> '.join(path)} with total distance {distance}")
    else:
        print(f"No path found from {start_node} to {end_node}")

if __name__ == "__main__":
    main()

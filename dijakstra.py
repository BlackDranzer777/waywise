from collections import deque
import heapq

def dijkstra(graph, start):
    # Initialize distances with infinity
    distances = {}
    for node in graph:
        distances[node] = float('inf')
    distances[start] = 0
    
    # Priority queue to hold nodes to visit
    priority_queue = [(0, start)]  # (distance, node)
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If the current distance is greater than the recorded distance, continue
        if current_distance > distances[current_node]:
            continue
        
        # Explore neighbors
        for neighbor in graph[current_node]:
            weight = graph[current_node][neighbor]
            distance = current_distance + weight
            
            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return distances

# Example usage
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

start_node = 'A'
distances = dijkstra(graph, start_node)
print(distances)

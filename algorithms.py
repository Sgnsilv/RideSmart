import time
import math
import heapq

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points on the Earth in meters.
    """
    R = 6371000.0  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    
    return R * c

def dijkstra_simple(graph, start, end, weight_field='length'):
    """
    Dijkstra's shortest path algorithm using a simple dictionary to find the minimum distance.
    Complexity: O(V^2)
    """
    start_time = time.perf_counter()
    nodes_expanded = 0
    
    distances = {node: float('inf') for node in graph.nodes}
    distances[start] = 0
    predecessors = {node: None for node in graph.nodes}
    
    unvisited = set(graph.nodes)
    path_found = False
    
    while unvisited:
        # Find unvisited node with minimum distance (Linear Search)
        current_node = min(unvisited, key=lambda n: distances[n])
        
        if distances[current_node] == float('inf') or current_node == end:
            if current_node == end:
                path_found = True
            break
            
        unvisited.remove(current_node)
        nodes_expanded += 1
        
        for neighbor in graph.neighbors(current_node):
            if neighbor not in unvisited:
                continue
            weight = graph[current_node][neighbor].get(weight_field, float('inf'))
            new_dist = distances[current_node] + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                predecessors[neighbor] = current_node
                
    path = []
    if path_found or distances[end] != float('inf'):
        curr = end
        while curr is not None:
            path.append(curr)
            curr = predecessors[curr]
        path.reverse()
        
    execution_time = time.perf_counter() - start_time
    return {
        'path': path,
        'cost': distances[end] if distances[end] != float('inf') else None,
        'nodes_expanded': nodes_expanded,
        'execution_time': execution_time
    }

def dijkstra_heap(graph, start, end, weight_field='length'):
    """
    Dijkstra's shortest path algorithm using a Min-Heap (priority queue).
    Complexity: O((V+E) log V)
    """
    start_time = time.perf_counter()
    nodes_expanded = 0
    
    distances = {node: float('inf') for node in graph.nodes}
    distances[start] = 0
    predecessors = {node: None for node in graph.nodes}
    
    pq = [(0, start)]
    visited = set()
    path_found = False
    
    while pq:
        dist, current_node = heapq.heappop(pq)
        
        if current_node in visited:
            continue
        visited.add(current_node)
        nodes_expanded += 1
        
        if current_node == end:
            path_found = True
            break
            
        for neighbor in graph.neighbors(current_node):
            if neighbor in visited:
                continue
            weight = graph[current_node][neighbor].get(weight_field, float('inf'))
            new_dist = dist + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                predecessors[neighbor] = current_node
                heapq.heappush(pq, (new_dist, neighbor))
                
    path = []
    if path_found or distances[end] != float('inf'):
        curr = end
        while curr is not None:
            path.append(curr)
            curr = predecessors[curr]
        path.reverse()
        
    execution_time = time.perf_counter() - start_time
    return {
        'path': path,
        'cost': distances[end] if distances[end] != float('inf') else None,
        'nodes_expanded': nodes_expanded,
        'execution_time': execution_time
    }

def a_star(graph, start, end, weight_field='length', max_speed_mps=22.22):
    """
    A* shortest path algorithm using a Min-Heap and geographic Haversine distance heuristic.
    Complexity: O((V+E) log V)
    """
    start_time = time.perf_counter()
    nodes_expanded = 0
    
    end_node = graph.nodes[end]
    end_lat, end_lon = end_node['y'], end_node['x']
    
    def heuristic(node_id):
        n = graph.nodes[node_id]
        dist_m = haversine_distance(n['y'], n['x'], end_lat, end_lon)
        if weight_field == 'length':
            return dist_m
        else:
            # If optimizing for time, divide the geographic distance by maximum speed in m/s
            # to keep the heuristic admissible (always <= actual remaining travel time).
            return dist_m / max_speed_mps
            
    g_score = {node: float('inf') for node in graph.nodes}
    g_score[start] = 0
    
    f_score = {node: float('inf') for node in graph.nodes}
    f_score[start] = heuristic(start)
    
    predecessors = {node: None for node in graph.nodes}
    pq = [(f_score[start], start)]
    visited = set()
    path_found = False
    
    while pq:
        _, current_node = heapq.heappop(pq)
        
        if current_node in visited:
            continue
        visited.add(current_node)
        nodes_expanded += 1
        
        if current_node == end:
            path_found = True
            break
            
        for neighbor in graph.neighbors(current_node):
            if neighbor in visited:
                continue
            weight = graph[current_node][neighbor].get(weight_field, float('inf'))
            tentative_g = g_score[current_node] + weight
            if tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                predecessors[neighbor] = current_node
                f_score[neighbor] = tentative_g + heuristic(neighbor)
                heapq.heappush(pq, (f_score[neighbor], neighbor))
                
    path = []
    if path_found or g_score[end] != float('inf'):
        curr = end
        while curr is not None:
            path.append(curr)
            curr = predecessors[curr]
        path.reverse()
        
    execution_time = time.perf_counter() - start_time
    return {
        'path': path,
        'cost': g_score[end] if g_score[end] != float('inf') else None,
        'nodes_expanded': nodes_expanded,
        'execution_time': execution_time
    }

def bidirectional_dijkstra(graph, start, end, weight_field='length'):
    """
    Bidirectional Dijkstra's shortest path algorithm. 
    Searches forward from 'start' and backward from 'end' simultaneously.
    Complexity: O((V+E) log V)
    """
    start_time = time.perf_counter()
    nodes_expanded = 0
    
    if start == end:
        return {
            'path': [start],
            'cost': 0.0,
            'nodes_expanded': 0,
            'execution_time': time.perf_counter() - start_time
        }
        
    # Forward search structures
    dist_f = {node: float('inf') for node in graph.nodes}
    dist_f[start] = 0
    pred_f = {node: None for node in graph.nodes}
    pq_f = [(0, start)]
    visited_f = set()
    
    # Backward search structures
    dist_b = {node: float('inf') for node in graph.nodes}
    dist_b[end] = 0
    pred_b = {node: None for node in graph.nodes}
    pq_b = [(0, end)]
    visited_b = set()
    
    mu = float('inf')  # cost of the best path found so far
    intersection_node = None
    
    while pq_f and pq_b:
        # Step forward
        if pq_f:
            d_f, u = heapq.heappop(pq_f)
            if u not in visited_f:
                visited_f.add(u)
                nodes_expanded += 1
                
                # Pruning condition
                if d_f + min(pq_b)[0] >= mu if pq_b else False:
                    break
                    
                for v in graph.successors(u):
                    if v in visited_f:
                        continue
                    w = graph[u][v].get(weight_field, float('inf'))
                    if dist_f[u] + w < dist_f[v]:
                        dist_f[v] = dist_f[u] + w
                        pred_f[v] = u
                        heapq.heappush(pq_f, (dist_f[v], v))
                        
                        # If node is visited by backward search, check if this forms a shorter path
                        if dist_b[v] != float('inf'):
                            if dist_f[v] + dist_b[v] < mu:
                                mu = dist_f[v] + dist_b[v]
                                intersection_node = v
                                
        # Step backward
        if pq_b:
            d_b, u = heapq.heappop(pq_b)
            if u not in visited_b:
                visited_b.add(u)
                nodes_expanded += 1
                
                # Pruning condition
                if d_b + min(pq_f)[0] >= mu if pq_f else False:
                    break
                    
                # In backward search, we follow incoming edges (predecessors in DiGraph)
                for v in graph.predecessors(u):
                    if v in visited_b:
                        continue
                    w = graph[v][u].get(weight_field, float('inf'))
                    if dist_b[u] + w < dist_b[v]:
                        dist_b[v] = dist_b[u] + w
                        pred_b[v] = u
                        heapq.heappush(pq_b, (dist_b[v], v))
                        
                        # If node is visited by forward search, check if this forms a shorter path
                        if dist_f[v] != float('inf'):
                            if dist_f[v] + dist_b[v] < mu:
                                mu = dist_f[v] + dist_b[v]
                                intersection_node = v
                                
        # Meeting condition check: if the top elements sum up to >= mu, we can stop
        if pq_f and pq_b:
            if pq_f[0][0] + pq_b[0][0] >= mu:
                break
                
    # Reconstruct the path if we found a connection
    path = []
    if intersection_node is not None and mu != float('inf'):
        # Forward path: from start to intersection_node
        curr = intersection_node
        f_path = []
        while curr is not None:
            f_path.append(curr)
            curr = pred_f[curr]
        f_path.reverse()
        
        # Backward path: from intersection_node to end (using pred_b)
        curr = pred_b[intersection_node]
        b_path = []
        while curr is not None:
            b_path.append(curr)
            curr = pred_b[curr]
            
        path = f_path + b_path
        
    execution_time = time.perf_counter() - start_time
    return {
        'path': path,
        'cost': mu if mu != float('inf') else None,
        'nodes_expanded': nodes_expanded,
        'execution_time': execution_time
    }

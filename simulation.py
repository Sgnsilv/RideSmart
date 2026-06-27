import osmnx as ox
import networkx as nx
import random
import time
from algorithms import dijkstra_heap, haversine_distance

# Set random seed for reproducibility
random.seed(42)

def download_and_prepare_graph(center_coords=(-5.8422, -35.2023), dist=2500):
    """
    Downloads the driving graph around the specified center coordinates,
    converts it to a simplified directed graph, and computes initial attributes.
    """
    print(f"Downloading graph from OSM for coordinates {center_coords} (radius {dist}m)...")
    # Download driving graph
    G_osm = ox.graph_from_point(center_coords, dist=dist, network_type='drive')
    print(f"Downloaded graph with {len(G_osm.nodes)} nodes and {len(G_osm.edges)} edges.")
    
    # Convert MultiDiGraph to simple DiGraph
    G = nx.DiGraph()
    
    # Copy nodes with their coordinates
    for node, data in G_osm.nodes(data=True):
        G.add_node(node, x=data.get('x'), y=data.get('y'))
        
    # Process edges and select shortest where multiple exist between same nodes
    for u, v, k, data in G_osm.edges(keys=True, data=True):
        length = float(data.get('length', 1.0))
        
        # Handle speed limits
        maxspeed_attr = data.get('maxspeed', 30.0)
        if isinstance(maxspeed_attr, list):
            speeds = []
            for s in maxspeed_attr:
                try:
                    speeds.append(float(s))
                except ValueError:
                    digits = ''.join(c for c in str(s) if c.isdigit())
                    if digits:
                        speeds.append(float(digits))
            maxspeed = max(speeds) if speeds else 30.0
        elif isinstance(maxspeed_attr, str):
            digits = ''.join(c for c in maxspeed_attr if c.isdigit())
            maxspeed = float(digits) if digits else 30.0
        else:
            try:
                maxspeed = float(maxspeed_attr)
            except (ValueError, TypeError):
                maxspeed = 30.0
                
        # Assign defaults based on road type if speed is default 30
        highway = data.get('highway', '')
        if maxspeed == 30.0:
            if 'primary' in str(highway):
                maxspeed = 60.0
            elif 'secondary' in str(highway):
                maxspeed = 50.0
            elif 'tertiary' in str(highway):
                maxspeed = 40.0
                
        speed_mps = maxspeed / 3.6
        time_free_flow = length / speed_mps
        
        # Add edge or update if shorter
        if G.has_edge(u, v):
            existing_data = G[u][v]
            if length < existing_data.get('length', float('inf')):
                G[u][v]['length'] = length
                G[u][v]['maxspeed'] = maxspeed
                G[u][v]['time_free_flow'] = time_free_flow
                G[u][v]['time_traffic'] = time_free_flow
                G[u][v]['geometry'] = data.get('geometry')
        else:
            G.add_edge(u, v,
                       length=length,
                       maxspeed=maxspeed,
                       time_free_flow=time_free_flow,
                       time_traffic=time_free_flow,
                       geometry=data.get('geometry'))
                       
    print(f"Simplified graph to DiGraph: {len(G.nodes)} nodes and {len(G.edges)} edges.")
    return G

def generate_synthetic_traffic(G, congestion_center=(-5.8422, -35.2023), radius=1000):
    """
    Applies synthetic traffic factors to the graph. 
    Uses a combination of a random basal factor and a distance-based congestion zone.
    """
    center_lat, center_lon = congestion_center
    
    for u, v, data in G.edges(data=True):
        # 1. Basal congestion (1.0 to 1.25 multiplier)
        basal_factor = random.uniform(1.0, 1.25)
        
        # 2. Localized congestion zone
        # Compute distance of node u to the congestion center
        u_lat, u_lon = G.nodes[u]['y'], G.nodes[u]['x']
        dist_to_center = haversine_distance(u_lat, u_lon, center_lat, center_lon)
        
        zone_factor = 1.0
        if dist_to_center < radius:
            # Linear decay: factor reaches +2.0 at the exact center and decays to +0 at the radius boundary
            zone_factor += (1.0 - (dist_to_center / radius)) * 2.0
            
        congestion_multiplier = basal_factor * zone_factor
        data['congestion_factor'] = congestion_multiplier
        data['time_traffic'] = data['time_free_flow'] * congestion_multiplier
        
    return G

def find_candidate_pickup_points(G, start_node, max_walk_distance):
    """
    Finds all nodes in G reachable from start_node within max_walk_distance.
    Returns a dictionary of node: distance_from_start.
    """
    # Custom Dijkstra bounded by max_walk_distance
    distances = {node: float('inf') for node in G.nodes}
    distances[start_node] = 0
    
    import heapq
    pq = [(0, start_node)]
    visited = set()
    
    candidates = {}
    
    while pq:
        dist, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        visited.add(u)
        
        if dist > max_walk_distance:
            continue
            
        candidates[u] = dist
        
        # Pedestrians can walk in both directions on any street (undirected walking)
        neighbors = []
        for v in G.successors(u):
            neighbors.append((v, G[u][v].get('length', float('inf'))))
        for v in G.predecessors(u):
            neighbors.append((v, G[v][u].get('length', float('inf'))))
            
        for v, w in neighbors:
            if v in visited:
                continue
            new_dist = dist + w
            if new_dist <= max_walk_distance and new_dist < distances[v]:
                distances[v] = new_dist
                heapq.heappush(pq, (new_dist, v))
                
    return candidates

def find_best_pickup_and_route(G, start, end, max_walk_distance, walk_speed_mps=1.2, weight_field='time_traffic', path_algorithm=dijkstra_heap):
    """
    Determines the best pickup node P that minimizes the total cost (walk to P + drive to end).
    Returns path details and a comparison to the no-walk case.
    """
    # 1. Find all reachable pickup nodes P
    candidates = find_candidate_pickup_points(G, start, max_walk_distance)
    
    best_p = None
    best_total_cost = float('inf')
    best_walk_cost = 0.0
    best_drive_cost = 0.0
    best_drive_path = []
    
    # 2. Evaluate each candidate
    for p, walk_dist in candidates.items():
        # Solve vehicle route from P to end
        result = path_algorithm(G, p, end, weight_field=weight_field)
        drive_path = result['path']
        drive_cost = result['cost']
        
        if drive_cost is None:
            continue
            
        # Calculate costs based on optimization field
        if weight_field == 'length':
            walk_cost = walk_dist
            total_cost = walk_cost + drive_cost
        else: # optimizing for time (free flow or traffic)
            walk_cost = walk_dist / walk_speed_mps
            total_cost = walk_cost + drive_cost
            
        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_p = p
            best_walk_cost = walk_cost
            best_drive_cost = drive_cost
            best_drive_path = drive_path
            
    # 3. Solve no-walk scenario (directly start -> end by car)
    no_walk_result = path_algorithm(G, start, end, weight_field=weight_field)
    no_walk_cost = no_walk_result['cost']
    no_walk_path = no_walk_result['path']
    
    # Reconstruct walk path from start to best_p
    # Using simple Dijkstra to get the path start -> best_p
    walk_path = []
    if best_p is not None:
        walk_result = path_algorithm(G, start, best_p, weight_field='length')
        walk_path = walk_result['path']
        
    return {
        'best_pickup_node': best_p,
        'walk_path': walk_path,
        'walk_cost': best_walk_cost,
        'drive_path': best_drive_path,
        'drive_cost': best_drive_cost,
        'total_cost': best_total_cost,
        'no_walk_path': no_walk_path,
        'no_walk_cost': no_walk_cost,
        'gain': (no_walk_cost - best_total_cost) if (no_walk_cost is not None and best_total_cost != float('inf')) else None
    }

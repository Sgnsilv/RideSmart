import osmnx as ox
import networkx as nx
import random
import time
import heapq
from scipy.spatial import KDTree
from algorithms import dijkstra_heap, haversine_distance

# Set random seed for reproducibility
random.seed(42)

def prepare_driving_graph(G_osm):
    """
    Converts a downloaded OSM MultiDiGraph to a simplified directed graph,
    and computes initial attributes like speed, time_free_flow, time_traffic.
    """
    # Convert MultiDiGraph to simple DiGraph
    G = nx.DiGraph()
    G.graph.update(G_osm.graph)
    
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

def download_and_prepare_graph(center_coords=(-5.8422, -35.2023), dist=2500):
    """
    Downloads the driving graph around the specified center coordinates,
    converts it to a simplified directed graph, and computes initial attributes.
    """
    print(f"Downloading graph from OSM for coordinates {center_coords} (radius {dist}m)...")
    # Download driving graph
    G_osm = ox.graph_from_point(center_coords, dist=dist, network_type='drive')
    print(f"Downloaded graph with {len(G_osm.nodes)} nodes and {len(G_osm.edges)} edges.")
    return prepare_driving_graph(G_osm)

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
    
    # Track the best alternative pickup point where the user actually walks (p != start)
    best_alt_p = None
    best_alt_total_cost = float('inf')
    best_alt_walk_cost = 0.0
    best_alt_drive_cost = 0.0
    best_alt_drive_path = []
    
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
            
        if p != start:
            if total_cost < best_alt_total_cost:
                best_alt_total_cost = total_cost
                best_alt_p = p
                best_alt_walk_cost = walk_cost
                best_alt_drive_cost = drive_cost
                best_alt_drive_path = drive_path
            
    # 3. Solve no-walk scenario (directly start -> end by car)
    no_walk_result = path_algorithm(G, start, end, weight_field=weight_field)
    no_walk_cost = no_walk_result['cost']
    no_walk_path = no_walk_result['path']
    
    # Reconstruct walk path from start to best_p
    walk_path = []
    if best_p is not None:
        walk_result = path_algorithm(G, start, best_p, weight_field='length')
        walk_path = walk_result['path']
        
    # Reconstruct alternative walk path from start to best_alt_p
    alt_walk_path = []
    if best_alt_p is not None:
        alt_walk_result = path_algorithm(G, start, best_alt_p, weight_field='length')
        alt_walk_path = alt_walk_result['path']
        
    return {
        'best_pickup_node': best_p,
        'walk_path': walk_path,
        'walk_cost': best_walk_cost,
        'drive_path': best_drive_path,
        'drive_cost': best_drive_cost,
        'total_cost': best_total_cost,
        
        'best_alt_pickup_node': best_alt_p,
        'alt_walk_path': alt_walk_path,
        'alt_walk_cost': best_alt_walk_cost,
        'alt_drive_path': best_alt_drive_path,
        'alt_drive_cost': best_alt_drive_cost,
        'alt_total_cost': best_alt_total_cost,
        
        'no_walk_path': no_walk_path,
        'no_walk_cost': no_walk_cost,
        'gain': (no_walk_cost - best_total_cost) if (no_walk_cost is not None and best_total_cost != float('inf')) else None
    }

def prepare_walking_graph(G_osm_walk):
    """
    Creates a clean DiGraph for walking where length is the primary weight
    and walk direction is modeled bidirectionally on every edge.
    """
    G = nx.DiGraph()
    G.graph.update(G_osm_walk.graph)
    for node, data in G_osm_walk.nodes(data=True):
        G.add_node(node, x=data.get('x'), y=data.get('y'))
        
    for u, v, k, data in G_osm_walk.edges(keys=True, data=True):
        length = float(data.get('length', 1.0))
        # Pedestrians can walk in both directions.
        # Keep the shortest edge if duplicates exist.
        if G.has_edge(u, v):
            if length < G[u][v]['length']:
                G[u][v]['length'] = length
        else:
            G.add_edge(u, v, length=length)
            
        if G.has_edge(v, u):
            if length < G[v][u]['length']:
                G[v][u]['length'] = length
        else:
            G.add_edge(v, u, length=length)
            
    return G

def download_multimodal_graphs(center_coords=(-5.8422, -35.2023), dist=2500):
    """
    Downloads both driving and walking MultiDiGraphs from OSM,
    simplifies them, and returns (G_drive, G_walk).
    """
    print(f"Downloading driving graph from OSM for coordinates {center_coords} (radius {dist}m)...")
    G_osm_drive = ox.graph_from_point(center_coords, dist=dist, network_type='drive')
    print(f"Downloaded driving graph with {len(G_osm_drive.nodes)} nodes and {len(G_osm_drive.edges)} edges.")
    G_drive = prepare_driving_graph(G_osm_drive)
    
    print(f"Downloading walking graph from OSM for coordinates {center_coords} (radius {dist}m)...")
    G_osm_walk = ox.graph_from_point(center_coords, dist=dist, network_type='walk')
    print(f"Downloaded walking graph with {len(G_osm_walk.nodes)} nodes and {len(G_osm_walk.edges)} edges.")
    G_walk = prepare_walking_graph(G_osm_walk)
    
    return G_drive, G_walk

def build_transfer_mapping(G_walk, G_drive, max_transfer_distance_m=30.0):
    """
    Maps each walkable node to its closest driving node if within max_transfer_distance_m.
    Returns a dictionary of walk_node: (drive_node, transfer_distance_meters).
    """
    drive_nodes = list(G_drive.nodes)
    drive_coords = [(G_drive.nodes[node]['y'], G_drive.nodes[node]['x']) for node in drive_nodes]
    drive_tree = KDTree(drive_coords)
    
    transfer_mapping = {}
    
    for walk_node in G_walk.nodes:
        walk_lat = G_walk.nodes[walk_node]['y']
        walk_lon = G_walk.nodes[walk_node]['x']
        
        # Fast KDTree geographic query
        dist_degrees, idx = drive_tree.query((walk_lat, walk_lon))
        closest_drive_node = drive_nodes[idx]
        
        # Calculate exact distance using Haversine formula
        drive_lat = G_drive.nodes[closest_drive_node]['y']
        drive_lon = G_drive.nodes[closest_drive_node]['x']
        dist_m = haversine_distance(walk_lat, walk_lon, drive_lat, drive_lon)
        
        if dist_m <= max_transfer_distance_m:
            transfer_mapping[walk_node] = (closest_drive_node, dist_m)
            
    return transfer_mapping

def find_candidate_pickup_points_multimodal(G_walk, start_walk_node, max_walk_distance, transfer_mapping):
    """
    Finds reachable walk nodes on G_walk within max_walk_distance,
    and maps them to candidate vehicle pickup nodes on G_drive.
    Returns a dict mapping drive_node: {'walk_dist': total_dist, 'walk_node': w_node}.
    """
    distances = {node: float('inf') for node in G_walk.nodes}
    distances[start_walk_node] = 0
    pq = [(0.0, start_walk_node)]
    visited = set()
    walk_reachable = {}
    
    while pq:
        dist, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if dist > max_walk_distance:
            continue
        walk_reachable[u] = dist
        
        for v in G_walk.successors(u):
            if v in visited:
                continue
            weight = G_walk[u][v].get('length', float('inf'))
            new_dist = dist + weight
            if new_dist <= max_walk_distance and new_dist < distances[v]:
                distances[v] = new_dist
                heapq.heappush(pq, (new_dist, v))
                
    drive_candidates = {}
    for w_node, w_dist in walk_reachable.items():
        if w_node in transfer_mapping:
            drive_node, trans_dist = transfer_mapping[w_node]
            total_walk_dist = w_dist + trans_dist
            if total_walk_dist <= max_walk_distance:
                if drive_node not in drive_candidates or total_walk_dist < drive_candidates[drive_node]['walk_dist']:
                    drive_candidates[drive_node] = {
                        'walk_dist': total_walk_dist,
                        'walk_node': w_node
                    }
                    
    return drive_candidates

def find_best_pickup_and_route_multimodal(
    G_drive, G_walk, transfer_mapping, start_coords, end_coords, 
    max_walk_distance, walk_speed_mps=1.2, weight_field='time_traffic', 
    path_algorithm=dijkstra_heap
):
    """
    Determines the best pickup node P in G_drive that minimizes total travel cost.
    Walks on G_walk from start to P, and drives on G_drive from P to end.
    """
    # Find nearest nodes to coordinates
    start_walk_node = ox.distance.nearest_nodes(G_walk, X=start_coords[1], Y=start_coords[0])
    end_drive_node = ox.distance.nearest_nodes(G_drive, X=end_coords[1], Y=end_coords[0])
    start_drive_node = ox.distance.nearest_nodes(G_drive, X=start_coords[1], Y=start_coords[0])
    
    # Get candidate pickup nodes from walk graph
    candidates = find_candidate_pickup_points_multimodal(
        G_walk, start_walk_node, max_walk_distance, transfer_mapping
    )
    
    # Always include the nearest drive node to the origin as a direct candidate
    # (walk_cost = 0, i.e. board the car right at the origin). This ensures
    # a viable route exists even when max_walk_distance is very small.
    if start_drive_node not in candidates:
        candidates[start_drive_node] = {'walk_dist': 0.0, 'walk_node': start_walk_node}
    
    best_p = None
    best_p_walk = None
    best_total_cost = float('inf')
    best_walk_cost = 0.0
    best_drive_cost = 0.0
    best_drive_path = []
    
    best_alt_p = None
    best_alt_p_walk = None
    best_alt_total_cost = float('inf')
    best_alt_walk_cost = 0.0
    best_alt_drive_cost = 0.0
    best_alt_drive_path = []
    
    # Evaluate candidates
    for p_drive, info in candidates.items():
        walk_dist = info['walk_dist']
        w_node = info['walk_node']
        
        result = path_algorithm(G_drive, p_drive, end_drive_node, weight_field=weight_field)
        drive_path = result['path']
        drive_cost = result['cost']
        
        if drive_cost is None:
            continue
            
        if weight_field == 'length':
            walk_cost = walk_dist
            total_cost = walk_cost + drive_cost
        else:
            walk_cost = walk_dist / walk_speed_mps
            total_cost = walk_cost + drive_cost
            
        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_p = p_drive
            best_p_walk = w_node
            best_walk_cost = walk_cost
            best_drive_cost = drive_cost
            best_drive_path = drive_path
            
        if p_drive != start_drive_node:
            if total_cost < best_alt_total_cost:
                best_alt_total_cost = total_cost
                best_alt_p = p_drive
                best_alt_p_walk = w_node
                best_alt_walk_cost = walk_cost
                best_alt_drive_cost = drive_cost
                best_alt_drive_path = drive_path
                
    # Solve no-walk scenario
    no_walk_result = path_algorithm(G_drive, start_drive_node, end_drive_node, weight_field=weight_field)
    no_walk_cost = no_walk_result['cost']
    no_walk_path = no_walk_result['path']
    
    # Reconstruct walk paths on G_walk
    walk_path = []
    if best_p_walk is not None:
        walk_path_res = path_algorithm(G_walk, start_walk_node, best_p_walk, weight_field='length')
        walk_path = walk_path_res['path']
        
    alt_walk_path = []
    if best_alt_p_walk is not None:
        alt_walk_path_res = path_algorithm(G_walk, start_walk_node, best_alt_p_walk, weight_field='length')
        alt_walk_path = alt_walk_path_res['path']
        
    return {
        'best_pickup_node': best_p,
        'walk_path': walk_path,
        'walk_cost': best_walk_cost,
        'drive_path': best_drive_path,
        'drive_cost': best_drive_cost,
        'total_cost': best_total_cost,
        
        'best_alt_pickup_node': best_alt_p,
        'alt_walk_path': alt_walk_path,
        'alt_walk_cost': best_alt_walk_cost,
        'alt_drive_path': best_alt_drive_path,
        'alt_drive_cost': best_alt_drive_cost,
        'alt_total_cost': best_alt_total_cost,
        
        'no_walk_path': no_walk_path,
        'no_walk_cost': no_walk_cost,
        'gain': (no_walk_cost - best_total_cost) if (no_walk_cost is not None and best_total_cost != float('inf')) else None
    }

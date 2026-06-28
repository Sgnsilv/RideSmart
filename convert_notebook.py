#!/usr/bin/env python3
"""Convert RideSmart notebook from single-graph to fully multimodal approach."""
import json

with open('RideSmart_Notebook.ipynb') as f:
    nb = json.load(f)

def set_cell(idx, source_str):
    """Replace the source of a code cell."""
    nb['cells'][idx]['source'] = source_str.split('\n')
    # Fix: split loses the newlines, we need to add them back
    lines = source_str.split('\n')
    nb['cells'][idx]['source'] = [line + '\n' for line in lines[:-1]] + [lines[-1]]
    # Clear outputs
    nb['cells'][idx]['outputs'] = []
    nb['cells'][idx]['execution_count'] = None

# ========== CELL 2: Imports - add osmnx and scipy ==========
set_cell(2, """import time
import math
import heapq
import random
import osmnx as ox
import networkx as nx
import numpy as np
import pandas as pd
import folium
import matplotlib.pyplot as plt
from scipy.spatial import KDTree""")

# ========== CELL 5: Update markdown for multimodal ==========
nb['cells'][5]['source'] = [
    "## 3. Implementação dos Métodos de Simulação Multimodal e Preparação de Grafos\n",
    "\n",
    "Funções para preparar grafos separados de direção e caminhada, gerar tráfego sintético,\n",
    "construir o mapeamento de transferência entre as malhas, e encontrar o melhor ponto de embarque\n",
    "na abordagem multimodal (pedestre + carro)."
]

# ========== CELL 6: Full multimodal simulation functions ==========
set_cell(6, """def prepare_driving_graph(G_osm):
    \"\"\"
    Converte um MultiDiGraph OSM para um DiGraph simplificado,
    computando atributos como speed, time_free_flow e time_traffic.
    \"\"\"
    G = nx.DiGraph()
    G.graph.update(G_osm.graph)
    for node, data in G_osm.nodes(data=True):
        G.add_node(node, x=data.get('x'), y=data.get('y'))
    for u, v, k, data in G_osm.edges(keys=True, data=True):
        length = float(data.get('length', 1.0))
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
        if G.has_edge(u, v):
            if length < G[u][v].get('length', float('inf')):
                G[u][v]['length'] = length
                G[u][v]['maxspeed'] = maxspeed
                G[u][v]['time_free_flow'] = time_free_flow
                G[u][v]['time_traffic'] = time_free_flow
        else:
            G.add_edge(u, v, length=length, maxspeed=maxspeed,
                       time_free_flow=time_free_flow, time_traffic=time_free_flow)
    print(f"Grafo de direção simplificado: {len(G.nodes)} nós e {len(G.edges)} arestas.")
    return G

def prepare_walking_graph(G_osm_walk):
    \"\"\"
    Cria um DiGraph limpo para caminhada onde cada aresta é bidirecional
    e o peso é a distância em metros.
    \"\"\"
    G = nx.DiGraph()
    G.graph.update(G_osm_walk.graph)
    for node, data in G_osm_walk.nodes(data=True):
        G.add_node(node, x=data.get('x'), y=data.get('y'))
    for u, v, k, data in G_osm_walk.edges(keys=True, data=True):
        length = float(data.get('length', 1.0))
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
    print(f"Grafo de caminhada simplificado: {len(G.nodes)} nós e {len(G.edges)} arestas.")
    return G

def generate_synthetic_traffic(G, congestion_center=(-5.8422, -35.2023), radius=1000):
    \"\"\"
    Aplica fatores de tráfego sintéticos ao grafo de direção.
    Usa fator basal aleatório + zona de congestionamento localizada.
    \"\"\"
    center_lat, center_lon = congestion_center
    for u, v, data in G.edges(data=True):
        basal_factor = random.uniform(1.0, 1.25)
        u_lat, u_lon = G.nodes[u]['y'], G.nodes[u]['x']
        dist_to_center = haversine_distance(u_lat, u_lon, center_lat, center_lon)
        zone_factor = 1.0
        if dist_to_center < radius:
            zone_factor += (1.0 - (dist_to_center / radius)) * 2.0
        congestion_multiplier = basal_factor * zone_factor
        data['congestion_factor'] = congestion_multiplier
        data['time_traffic'] = data['time_free_flow'] * congestion_multiplier
    return G

def build_transfer_mapping(G_walk, G_drive, max_transfer_distance_m=30.0):
    \"\"\"
    Mapeia cada nó de caminhada ao nó de direção mais próximo (se dentro de max_transfer_distance_m).
    Usa KDTree para busca espacial eficiente.
    \"\"\"
    drive_nodes = list(G_drive.nodes)
    drive_coords = [(G_drive.nodes[node]['y'], G_drive.nodes[node]['x']) for node in drive_nodes]
    drive_tree = KDTree(drive_coords)
    transfer_mapping = {}
    for walk_node in G_walk.nodes:
        walk_lat = G_walk.nodes[walk_node]['y']
        walk_lon = G_walk.nodes[walk_node]['x']
        dist_degrees, idx = drive_tree.query((walk_lat, walk_lon))
        closest_drive_node = drive_nodes[idx]
        drive_lat = G_drive.nodes[closest_drive_node]['y']
        drive_lon = G_drive.nodes[closest_drive_node]['x']
        dist_m = haversine_distance(walk_lat, walk_lon, drive_lat, drive_lon)
        if dist_m <= max_transfer_distance_m:
            transfer_mapping[walk_node] = (closest_drive_node, dist_m)
    print(f"Mapeamento de transferência: {len(transfer_mapping)} nós de caminhada mapeados para nós de direção.")
    return transfer_mapping

def find_candidate_pickup_points_multimodal(G_walk, start_walk_node, max_walk_distance, transfer_mapping):
    \"\"\"
    Encontra nós alcançáveis a pé em G_walk dentro de max_walk_distance,
    e mapeia para candidatos de embarque em G_drive via transfer_mapping.
    \"\"\"
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
    \"\"\"
    Determina o melhor nó de embarque P em G_drive que minimiza o custo total.
    Caminha em G_walk de start até P, e dirige em G_drive de P até end.
    \"\"\"
    start_walk_node = ox.distance.nearest_nodes(G_walk, X=start_coords[1], Y=start_coords[0])
    end_drive_node = ox.distance.nearest_nodes(G_drive, X=end_coords[1], Y=end_coords[0])
    start_drive_node = ox.distance.nearest_nodes(G_drive, X=start_coords[1], Y=start_coords[0])

    candidates = find_candidate_pickup_points_multimodal(
        G_walk, start_walk_node, max_walk_distance, transfer_mapping
    )

    # Sempre incluir o nó de direção mais próximo da origem como candidato direto
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

    no_walk_result = path_algorithm(G_drive, start_drive_node, end_drive_node, weight_field=weight_field)
    no_walk_cost = no_walk_result['cost']
    no_walk_path = no_walk_result['path']

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
    }""")

# ========== CELL 7: Update markdown ==========
nb['cells'][7]['source'] = [
    "## 4. Inicialização e Simulação Multimodal em Natal/RN (UFRN)\n",
    "\n",
    "Baixamos dois grafos separados do OpenStreetMap (direção e caminhada), aplicamos tráfego sintético\n",
    "ao grafo de direção, e construímos o mapeamento de transferência entre as malhas."
]

# ========== CELL 8: Multimodal graph initialization ==========
set_cell(8, """ufrn_coords = (-5.8422, -35.2023)
graph_center = (-5.835, -35.210)

# Baixar grafos separados de direção e caminhada do OSM
G_osm_drive = ox.graph_from_point(graph_center, dist=2500, network_type='drive')
G_osm_walk = ox.graph_from_point(graph_center, dist=2500, network_type='walk')

# Preparar grafos simplificados
G_drive = prepare_driving_graph(G_osm_drive)
G_walk = prepare_walking_graph(G_osm_walk)

# Aplicar tráfego sintético ao grafo de direção
random.seed(42)
G_drive = generate_synthetic_traffic(G_drive, congestion_center=ufrn_coords, radius=1200)

# Construir mapeamento de transferência (caminhada -> direção)
transfer_mapping = build_transfer_mapping(G_walk, G_drive, max_transfer_distance_m=30.0)

# Visualização estatística
factors = [data['congestion_factor'] for u, v, data in G_drive.edges(data=True)]
print(f"\\nFatores de congestionamento: Mín: {min(factors):.2f}x | Méd: {np.mean(factors):.2f}x | Máx: {max(factors):.2f}x")
print(f"Nós G_drive: {len(G_drive.nodes)} | Arestas G_drive: {len(G_drive.edges)}")
print(f"Nós G_walk: {len(G_walk.nodes)} | Arestas G_walk: {len(G_walk.edges)}")""")

# ========== CELL 9: Update markdown ==========
nb['cells'][9]['source'] = [
    "## 5. Demonstração de Viagem Multimodal Otimizada\n",
    "\n",
    "Selecionamos dois pontos aleatórios e aplicamos a otimização multimodal\n",
    "(caminhada em G_walk + direção em G_drive) para encontrar o melhor ponto de embarque."
]

# ========== CELL 10: Demo with multimodal ==========
set_cell(10, """# Selecionar dois pontos aleatórios para demonstração
random.seed(101)
drive_nodes_list = list(G_drive.nodes)

while True:
    origin_node = random.choice(drive_nodes_list)
    dest_node = random.choice(drive_nodes_list)
    if origin_node != dest_node and nx.has_path(G_drive, origin_node, dest_node):
        break

origin_coords = (G_drive.nodes[origin_node]['y'], G_drive.nodes[origin_node]['x'])
dest_coords = (G_drive.nodes[dest_node]['y'], G_drive.nodes[dest_node]['x'])

max_walk_dist = 600.0  # 600m
walk_speed = 1.2       # 1.2 m/s

print(f"Origem (A): {origin_node} ({origin_coords[0]:.5f}, {origin_coords[1]:.5f})")
print(f"Destino (B): {dest_node} ({dest_coords[0]:.5f}, {dest_coords[1]:.5f})")
print(f"Distância máxima de caminhada: {max_walk_dist}m\\n")

result = find_best_pickup_and_route_multimodal(
    G_drive=G_drive, G_walk=G_walk, transfer_mapping=transfer_mapping,
    start_coords=origin_coords, end_coords=dest_coords,
    max_walk_distance=max_walk_dist, walk_speed_mps=walk_speed,
    weight_field='time_traffic', path_algorithm=dijkstra_heap
)

print("=== RESULTADOS ===")
if result['best_pickup_node'] is not None:
    print(f"Embarque ideal (P): {result['best_pickup_node']}")
    print(f"Distância de caminhada: {result['walk_cost'] * walk_speed:.1f} metros")
    print(f"Tempo de caminhada: {result['walk_cost']:.1f}s")
    print(f"Tempo de carro (P -> B): {result['drive_cost']:.1f}s")
    print(f"Tempo Total de viagem (Caminhada + Carro): {result['total_cost']:.1f}s ({(result['total_cost']/60):.2f} min)")

    print("\\n--- COMPARAÇÃO DE OPÇÕES ---")
    print(f"  1. Sem Caminhada (Embarque na Origem A): {result['no_walk_cost']:.1f}s ({(result['no_walk_cost']/60):.2f} min)")
    if result['best_alt_pickup_node'] is not None:
        print(f"  2. Com Caminhada (Melhor ponto alternativo P={result['best_alt_pickup_node']}):")
        print(f"     Caminhada: {result['alt_walk_cost']*walk_speed:.1f}m ({result['alt_walk_cost']:.1f}s) | Carro: {result['alt_drive_cost']:.1f}s")
        print(f"     Tempo Total: {result['alt_total_cost']:.1f}s ({(result['alt_total_cost']/60):.2f} min)")

    print("\\n--- CONCLUSÃO ---")
    if result['gain'] is not None and result['gain'] > 0:
        print(f"Economia ao caminhar: {result['gain']:.1f}s ({(result['gain']/60):.2f} min) economizados!")
    else:
        print("Caminhar não trouxe economia nesta rota (melhor embarcar direto na origem).")""")

# ========== CELL 12: Folium map with multimodal ==========
set_cell(12, """m = folium.Map(location=ufrn_coords, zoom_start=15)

a_coords = origin_coords
b_coords = dest_coords

folium.Marker(location=a_coords, popup="Origem (A)", icon=folium.Icon(color='green', icon='user')).add_to(m)
folium.Marker(location=b_coords, popup="Destino (B)", icon=folium.Icon(color='red', icon='flag')).add_to(m)

if result['best_pickup_node'] is not None:
    p_coords = (G_drive.nodes[result['best_pickup_node']]['y'], G_drive.nodes[result['best_pickup_node']]['x'])
    folium.Marker(location=p_coords, popup="Embarque Otimizado (P)", icon=folium.Icon(color='purple', icon='car')).add_to(m)

    walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in result['walk_path']]
    folium.PolyLine(walk_line, color='blue', weight=4, opacity=0.8, tooltip="Caminhada").add_to(m)

    drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in result['drive_path']]
    folium.PolyLine(drive_line, color='red', weight=5, opacity=0.7, tooltip="Carro").add_to(m)
m""")

# ========== CELL 13: Update markdown for multimodal ==========
nb['cells'][13]['source'] = [
    "## 6.1 Análise de Casos de Estudo Multimodais (Locais Reais de Natal)\n",
    "\n",
    "Para ilustrar de forma prática a dinâmica da caminhada multimodal e os efeitos do trânsito na escolha\n",
    "do melhor local de embarque, analisamos 3 rotas entre pontos reais de Natal utilizando os grafos\n",
    "separados de caminhada e direção e o mapeamento de transferência."
]

# ========== CELL 14: Case study 1 - multimodal ==========
set_cell(14, """# Coordenadas de Landmarks reais de Natal
landmarks_coords = {
    'ECT': (-5.8437, -35.2013),
    'Midway Mall': (-5.8118, -35.2052),
    'Natal Shopping': (-5.8427, -35.2100),
    'Havan': (-5.8166, -35.2120),
    'Reitoria': (-5.8422, -35.2023),
    'CT (Centro de Tecnologia)': (-5.8427, -35.210)
}

# CENÁRIO 1: Caminhada Altamente Vantajosa (Reitoria -> Midway Mall)
print("=== CENÁRIO 1: Reitoria -> Midway Mall ===")
res1 = find_best_pickup_and_route_multimodal(
    G_drive=G_drive, G_walk=G_walk, transfer_mapping=transfer_mapping,
    start_coords=landmarks_coords['Reitoria'], end_coords=landmarks_coords['Midway Mall'],
    max_walk_distance=600.0, walk_speed_mps=1.2, weight_field='time_traffic'
)

print(f"Ponto de Embarque Ideal (P): {res1['best_pickup_node']}")
print(f"Distância de caminhada: {res1['walk_cost'] * 1.2:.1f} metros")
print(f"Tempo de caminhada: {res1['walk_cost']:.1f}s")
print(f"Tempo de carro (P -> B): {res1['drive_cost']:.1f}s")
print(f"Tempo total (Caminhada + Carro): {res1['total_cost']:.1f}s ({(res1['total_cost']/60):.2f} min)")
print(f"Sem caminhar (Carro A -> B): {res1['no_walk_cost']:.1f}s ({(res1['no_walk_cost']/60):.2f} min)")
if res1['gain'] is not None and res1['gain'] > 0:
    print(f"Economia ao caminhar: {res1['gain']:.1f}s ({(res1['gain']/60):.2f} min) economizados!")
else:
    print("Caminhar não trouxe economia nesta rota.")

# Plotagem do Cenário 1
m1 = folium.Map(location=(-5.835, -35.210), zoom_start=14)
folium.Marker(location=landmarks_coords['Reitoria'], popup="Origem (Reitoria)", icon=folium.Icon(color='green', icon='user')).add_to(m1)
folium.Marker(location=landmarks_coords['Midway Mall'], popup="Destino (Midway Mall)", icon=folium.Icon(color='red', icon='flag')).add_to(m1)
if res1['best_pickup_node'] is not None:
    p1 = res1['best_pickup_node']
    folium.Marker(location=(G_drive.nodes[p1]['y'], G_drive.nodes[p1]['x']), popup="Embarque Otimizado (P)", icon=folium.Icon(color='purple', icon='car')).add_to(m1)
    walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in res1['walk_path']]
    folium.PolyLine(walk_line, color='blue', weight=4, opacity=0.8, tooltip="Caminhada").add_to(m1)
    drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in res1['drive_path']]
    folium.PolyLine(drive_line, color='red', weight=5, opacity=0.7, tooltip="Carro").add_to(m1)
m1""")

# ========== CELL 16: Case study 2 - multimodal ==========
set_cell(16, """# CENÁRIO 2: Caminhada Ineficaz (Natal Shopping -> ECT)
print("=== CENÁRIO 2: Natal Shopping -> ECT ===")
res2 = find_best_pickup_and_route_multimodal(
    G_drive=G_drive, G_walk=G_walk, transfer_mapping=transfer_mapping,
    start_coords=landmarks_coords['Natal Shopping'], end_coords=landmarks_coords['ECT'],
    max_walk_distance=600.0, walk_speed_mps=1.2, weight_field='time_traffic'
)

print(f"Ponto de Embarque Ideal (P): {res2['best_pickup_node']}")
print(f"Distância de caminhada: {res2['walk_cost'] * 1.2:.1f} metros")
print(f"Tempo total (Caminhada + Carro): {res2['total_cost']:.1f}s ({(res2['total_cost']/60):.2f} min)")
print(f"Sem caminhar (Carro A -> B): {res2['no_walk_cost']:.1f}s ({(res2['no_walk_cost']/60):.2f} min)")
if res2['gain'] is not None and res2['gain'] > 0:
    print(f"Economia ao caminhar: {res2['gain']:.1f}s")
else:
    print("Análise: Caminhar NÃO trouxe economia - o embarque direto na origem é o mais eficiente.")

# Plotagem do Cenário 2
m2 = folium.Map(location=(-5.835, -35.210), zoom_start=14)
folium.Marker(location=landmarks_coords['Natal Shopping'], popup="Origem (Natal Shopping)", icon=folium.Icon(color='green', icon='user')).add_to(m2)
folium.Marker(location=landmarks_coords['ECT'], popup="Destino (ECT)", icon=folium.Icon(color='red', icon='flag')).add_to(m2)
if res2['best_pickup_node'] is not None:
    p2 = res2['best_pickup_node']
    folium.Marker(location=(G_drive.nodes[p2]['y'], G_drive.nodes[p2]['x']), popup="Embarque Otimizado (P)", icon=folium.Icon(color='purple', icon='car')).add_to(m2)
    walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in res2['walk_path']]
    folium.PolyLine(walk_line, color='blue', weight=4, opacity=0.8, tooltip="Caminhada").add_to(m2)
    drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in res2['drive_path']]
    folium.PolyLine(drive_line, color='red', weight=5, opacity=0.7, tooltip="Carro").add_to(m2)
m2""")

# ========== CELL 18: Case study 3 - multimodal ==========
set_cell(18, """# CENÁRIO 3: Sensibilidade ao Trânsito (CT UFRN -> Midway Mall)
print("=== CENÁRIO 3: CT UFRN -> Midway Mall ===")
res3_traffic = find_best_pickup_and_route_multimodal(
    G_drive=G_drive, G_walk=G_walk, transfer_mapping=transfer_mapping,
    start_coords=landmarks_coords['CT (Centro de Tecnologia)'], end_coords=landmarks_coords['Midway Mall'],
    max_walk_distance=600.0, walk_speed_mps=1.2, weight_field='time_traffic'
)
res3_free = find_best_pickup_and_route_multimodal(
    G_drive=G_drive, G_walk=G_walk, transfer_mapping=transfer_mapping,
    start_coords=landmarks_coords['CT (Centro de Tecnologia)'], end_coords=landmarks_coords['Midway Mall'],
    max_walk_distance=600.0, walk_speed_mps=1.2, weight_field='time_free_flow'
)

print(f"[FLUXO LIVRE] Embarque ideal: {res3_free['best_pickup_node']} (caminhada de {res3_free['walk_cost']*1.2:.1f}m)")
print(f"[TRÂNSITO]    Embarque ideal: {res3_traffic['best_pickup_node']} (caminhada de {res3_traffic['walk_cost']*1.2:.1f}m)")
if res3_traffic['gain'] is not None:
    print(f"              Ganho sob trânsito: {res3_traffic['gain']:.1f}s")
print("Análise: Sob trânsito pesado, o ponto de embarque se desloca para fora do ponto inicial,")
print("economizando tempo precioso por desviar de trechos congestionados.")

# Plotagem do Cenário 3 (Trânsito)
m3 = folium.Map(location=(-5.835, -35.210), zoom_start=14)
folium.Marker(location=landmarks_coords['CT (Centro de Tecnologia)'], popup="Origem (CT)", icon=folium.Icon(color='green', icon='user')).add_to(m3)
folium.Marker(location=landmarks_coords['Midway Mall'], popup="Destino (Midway)", icon=folium.Icon(color='red', icon='flag')).add_to(m3)
if res3_traffic['best_pickup_node'] is not None:
    p3 = res3_traffic['best_pickup_node']
    folium.Marker(location=(G_drive.nodes[p3]['y'], G_drive.nodes[p3]['x']), popup="Embarque Otimizado (P)", icon=folium.Icon(color='purple', icon='car')).add_to(m3)
    walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in res3_traffic['walk_path']]
    folium.PolyLine(walk_line, color='blue', weight=4, opacity=0.8, tooltip="Caminhada").add_to(m3)
    drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in res3_traffic['drive_path']]
    folium.PolyLine(drive_line, color='red', weight=5, opacity=0.7, tooltip="Carro").add_to(m3)
m3""")

# ========== CELL 20: Benchmark with multimodal ==========
set_cell(20, """# Benchmark: 50 rotas aleatórias comparando os 4 algoritmos
benchmark_pairs = []
random.seed(42)
drive_nodes_list = list(G_drive.nodes)

while len(benchmark_pairs) < 50:
    s = random.choice(drive_nodes_list)
    t = random.choice(drive_nodes_list)
    if s != t and nx.has_path(G_drive, s, t):
        benchmark_pairs.append((s, t))

metrics = {
    'Dijkstra Simples': {'times': [], 'nodes': []},
    'Dijkstra Heap': {'times': [], 'nodes': []},
    'A*': {'times': [], 'nodes': []},
    'Dijkstra Bidirecional': {'times': [], 'nodes': []}
}

for s, t in benchmark_pairs:
    # Dijkstra Simples
    res_simple = dijkstra_simple(G_drive, s, t, weight_field='time_traffic')
    metrics['Dijkstra Simples']['times'].append(res_simple['execution_time'])
    metrics['Dijkstra Simples']['nodes'].append(res_simple['nodes_expanded'])

    # Dijkstra Heap
    res_heap = dijkstra_heap(G_drive, s, t, weight_field='time_traffic')
    metrics['Dijkstra Heap']['times'].append(res_heap['execution_time'])
    metrics['Dijkstra Heap']['nodes'].append(res_heap['nodes_expanded'])

    # A*
    res_astar = a_star(G_drive, s, t, weight_field='time_traffic', max_speed_mps=19.44)
    metrics['A*']['times'].append(res_astar['execution_time'])
    metrics['A*']['nodes'].append(res_astar['nodes_expanded'])

    # Dijkstra Bidirecional
    res_bidir = bidirectional_dijkstra(G_drive, s, t, weight_field='time_traffic')
    metrics['Dijkstra Bidirecional']['times'].append(res_bidir['execution_time'])
    metrics['Dijkstra Bidirecional']['nodes'].append(res_bidir['nodes_expanded'])

summary_df = pd.DataFrame(index=metrics.keys())
summary_df['Tempo Médio (ms)'] = [np.mean(metrics[alg]['times']) * 1000 for alg in metrics]
summary_df['Nós Expandidos Médios'] = [np.mean(metrics[alg]['nodes']) for alg in metrics]
summary_df['Nós Expandidos Mínimos'] = [np.min(metrics[alg]['nodes']) for alg in metrics]
summary_df['Nós Expandidos Máximos'] = [np.max(metrics[alg]['nodes']) for alg in metrics]
summary_df""")

# ========== CELL 24: Playground - already multimodal, just ensure it works ==========
set_cell(24, """# ==================================================================
# DEFINA AQUI OS PARÂMETROS DA CONSULTA INTERATIVA (TESTE NA HORA):
# ==================================================================
origem = (-5.8427, -35.210)       # (Latitude, Longitude) do ponto A (CT UFRN)
destino = (-5.8118, -35.2052)     # (Latitude, Longitude) do ponto B (Midway Mall)
caminhada_maxima = 300.0          # Distância máxima de caminhada (metros)
# ==================================================================

# Executar a otimização multimodal com os parâmetros digitados
resultado = find_best_pickup_and_route_multimodal(
    G_drive=G_drive,
    G_walk=G_walk,
    transfer_mapping=transfer_mapping,
    start_coords=origem,
    end_coords=destino,
    max_walk_distance=caminhada_maxima,
    walk_speed_mps=1.2,
    weight_field='time_traffic',
    path_algorithm=dijkstra_heap
)

# Exibir os resultados formatados
if resultado['best_pickup_node'] and resultado['drive_path']:
    print("=================== RESULTADOS DA ROTA ===================")
    print(f"Origem (A):                   {origem}")
    print(f"Ponto de Embarque Ideal (P):  Nó {resultado['best_pickup_node']}")
    print(f"Destino (B):                  {destino}")
    print("----------------------------------------------------------")
    print(f"Distância de Caminhada (A->P): {resultado['walk_cost'] * 1.2:.2f} metros")
    print(f"Tempo de Caminhada (A->P):     {resultado['walk_cost']:.1f} segundos ({resultado['walk_cost']/60:.2f} min)")
    print(f"Tempo de Carro (P->B):         {resultado['drive_cost']:.1f} segundos ({resultado['drive_cost']/60:.2f} min)")
    print("----------------------------------------------------------")
    print(f"Tempo TOTAL de Viagem:         {resultado['total_cost']:.1f} segundos ({resultado['total_cost']/60:.2f} min)")
    print("==========================================================")
    
    # Plotar o mapa interativo na tela
    fig, ax = plt.subplots(figsize=(10, 10), facecolor='white')
    ax.set_facecolor('white')
    
    # Plotar malha viária como fundo
    for u, v, d_data in G_drive.edges(data=True):
        x1, y1 = G_drive.nodes[u]['x'], G_drive.nodes[u]['y']
        x2, y2 = G_drive.nodes[v]['x'], G_drive.nodes[v]['y']
        ax.plot([x1, x2], [y1, y2], color='#e0e0e0', linewidth=0.8, zorder=3)
    
    # Plotar trecho a pé (azul)
    walk_path = resultado['walk_path']
    if walk_path:
        w_lats = [G_walk.nodes[n]['y'] for n in walk_path]
        w_lons = [G_walk.nodes[n]['x'] for n in walk_path]
        ax.plot(w_lons, w_lats, color='#1e88e5', linewidth=4.5, label='Caminhada (Pedestre)', zorder=5)
        
    # Plotar trecho de carro (vermelho)
    drive_path = resultado['drive_path']
    if drive_path:
        d_lats = [G_drive.nodes[n]['y'] for n in drive_path]
        d_lons = [G_drive.nodes[n]['x'] for n in drive_path]
        ax.plot(d_lons, d_lats, color='#e53935', linewidth=3.5, label='Trajeto de Carro', zorder=4)
        
    # Plotar marcadores especiais
    ax.scatter(origem[1], origem[0], color='#4caf50', s=150, zorder=6, label='Origem (A)')
    ax.scatter(destino[1], destino[0], color='#f44336', s=150, zorder=6, label='Destino (B)')
    
    p_node = resultado['best_pickup_node']
    p_lat, p_lon = G_drive.nodes[p_node]['y'], G_drive.nodes[p_node]['x']
    ax.scatter(p_lon, p_lat, color='#8e24aa', s=200, marker='*', zorder=7, label='Embarque (P)')
    
    ax.legend(loc='upper left', fontsize=12)
    ax.set_title(f"RideSmart - Rota Multimodal Otimizada (Total: {resultado['total_cost']/60:.2f} min)")
    plt.show()
else:
    print("[Erro] Não foi possível traçar uma rota válida com esses parâmetros. Verifique se o limite de caminhada não é muito baixo.")""")

# Save the modified notebook
with open('RideSmart_Notebook.ipynb', 'w') as f:
    json.dump(nb, f, indent=1, ensure_ascii=False)

print("Notebook convertido para multimodal com sucesso!")

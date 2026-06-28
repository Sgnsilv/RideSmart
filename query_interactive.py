import os
import sys
import matplotlib.pyplot as plt
import osmnx as ox

from simulation import (
    prepare_driving_graph,
    prepare_walking_graph,
    generate_synthetic_traffic,
    build_transfer_mapping,
    find_best_pickup_and_route_multimodal
)
from algorithms import dijkstra_heap

def run_custom_query():
    print("==================================================")
    print("          RideSmart - Consulta Interativa         ")
    print("==================================================")
    
    # 1. Carregar coordenadas da UFRN
    ufrn_coords = (-5.8422, -35.2023)
    graph_center = (-5.835, -35.210)
    
    print("\n[1/3] Baixando e preparando grafos da região...")
    G_osm_drive = ox.graph_from_point(graph_center, dist=2500, network_type='drive')
    G_osm_walk = ox.graph_from_point(graph_center, dist=2500, network_type='walk')
    
    G_drive = prepare_driving_graph(G_osm_drive)
    G_walk = prepare_walking_graph(G_osm_walk)
    G_drive = generate_synthetic_traffic(G_drive, congestion_center=ufrn_coords, radius=1200)
    transfer_mapping = build_transfer_mapping(G_walk, G_drive, max_transfer_distance_m=30.0)
    print("Grafos preparados com sucesso!")
    
    # 2. Entrar com dados de entrada
    print("\n[2/3] Defina os parâmetros da simulação:")
    
    # Exemplos rápidos de pontos
    print("Exemplos úteis de Coordenadas de Referência UFRN:")
    print("  - CT (Centro de Tecnologia): -5.8427, -35.210")
    print("  - Reitoria UFRN:             -5.8422, -35.2023")
    print("  - ECT (Escola de Ciências):  -5.8437, -35.2013")
    print("  - Biblioteca Central:        -5.8420, -35.2005")
    print("  - Midway Mall (Shopping):    -5.8118, -35.2052")
    print("  - Natal Shopping:            -5.8427, -35.2100")
    
    try:
        lat_a, lon_a = map(float, input("\nDigite a Latitude e Longitude de ORIGEM (A) (ex: -5.8427, -35.210): ").split(","))
        lat_b, lon_b = map(float, input("Digite a Latitude e Longitude de DESTINO (B) (ex: -5.8118, -35.2052): ").split(","))
        max_walk = float(input("Digite a distância máxima de caminhada permitida X (metros) (ex: 300): "))
    except ValueError:
        print("\n[Erro] Entrada inválida. Usando valores padrão (CT -> Midway Mall, X=300m).")
        lat_a, lon_a = -5.8427, -35.210
        lat_b, lon_b = -5.8118, -35.2052
        max_walk = 300.0
        
    print("\n[3/3] Calculando a rota multimodal ótima...")
    result = find_best_pickup_and_route_multimodal(
        G_drive=G_drive,
        G_walk=G_walk,
        transfer_mapping=transfer_mapping,
        start_coords=(lat_a, lon_a),
        end_coords=(lat_b, lon_b),
        max_walk_distance=max_walk,
        walk_speed_mps=1.2,
        weight_field='time_traffic',
        path_algorithm=dijkstra_heap
    )
    
    if not result['best_pickup_node'] or not result['drive_path']:
        print("\n[Erro] Não foi possível encontrar uma rota viável com esses parâmetros.")
        return
        
    # 3. Exibir resultados
    print("\n=================== RESULTADOS ===================")
    print("Ponto de Origem (A):       ({:.5f}, {:.5f})".format(lat_a, lon_a))
    print("Ponto de Embarque (P):     Nó {}".format(result['best_pickup_node']))
    print("Ponto de Destino (B):      ({:.5f}, {:.5f})".format(lat_b, lon_b))
    print("--------------------------------------------------")
    print("Distância de Caminhada (A->P):  {:.2f} metros".format(result['walk_cost'] * 1.2))
    print("Tempo de Caminhada (A->P):      {:.1f} segundos ({:.2f} minutos)".format(result['walk_cost'], result['walk_cost']/60.0))
    print("Tempo de Carro (P->B):          {:.1f} segundos ({:.2f} minutos)".format(result['drive_cost'], result['drive_cost']/60.0))
    print("--------------------------------------------------")
    print("Tempo TOTAL de Viagem:          {:.1f} segundos ({:.2f} minutos)".format(result['total_cost'], result['total_cost']/60.0))
    print("==================================================")
    
    # 4. Gerar e salvar mapa
    print("\nPlotando e salvando mapa da rota...")
    fig, ax = plt.subplots(figsize=(10, 10), facecolor='white')
    ax.set_facecolor('white')
    
    # Plotar malha viária como fundo
    ox.plot_graph(G_osm_drive, ax=ax, node_size=0, edge_color='#e0e0e0', edge_linewidth=0.8, show=False, close=False)
    
    # Plotar rota de caminhada
    walk_path = result['walk_path']
    if walk_path:
        w_lats = [G_walk.nodes[n]['y'] for n in walk_path]
        w_lons = [G_walk.nodes[n]['x'] for n in walk_path]
        ax.plot(w_lons, w_lats, color='#1e88e5', linewidth=4.5, label='Trecho Caminhada', zorder=5)
        
    # Plotar rota de carro
    drive_path = result['drive_path']
    if drive_path:
        d_lats = [G_drive.nodes[n]['y'] for n in drive_path]
        d_lons = [G_drive.nodes[n]['x'] for n in drive_path]
        ax.plot(d_lons, d_lats, color='#e53935', linewidth=3.5, label='Trecho Carro', zorder=4)
        
    # Plotar pontos especiais
    ax.scatter(lon_a, lat_a, color='#4caf50', s=150, zorder=6, label='Origem (A)')
    ax.scatter(lon_b, lat_b, color='#f44336', s=150, zorder=6, label='Destino (B)')
    
    p_node = result['best_pickup_node']
    p_lat, p_lon = G_drive.nodes[p_node]['y'], G_drive.nodes[p_node]['x']
    ax.scatter(p_lon, p_lat, color='#8e24aa', s=200, marker='*', zorder=7, label='Ponto de Embarque (P)')
    
    ax.legend(loc='upper left', fontsize=11)
    ax.set_title("RideSmart - Rota Multimodal Otimizada\nTempo Total: {:.2f} min (Caminhada: {:.1f}m)".format(
        result['total_cost']/60.0, result['walk_cost'] * 1.2
    ))
    
    plt.tight_layout()
    map_name = "custom_route.png"
    plt.savefig(map_name, dpi=300)
    plt.close()
    
    print("\n>>> Mapa da rota salvo com sucesso como: '{}' <<<".format(os.path.abspath(map_name)))
    print("==================================================")

if __name__ == "__main__":
    run_custom_query()

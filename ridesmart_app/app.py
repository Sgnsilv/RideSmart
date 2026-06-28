import streamlit as st
import folium
from streamlit_folium import st_folium
import osmnx as ox
import networkx as nx
import sys
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Add parent directory to path so we can import simulation and algorithms
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation import (
    prepare_driving_graph,
    prepare_walking_graph,
    generate_synthetic_traffic,
    build_transfer_mapping,
    find_best_pickup_and_route_multimodal
)
from algorithms import dijkstra_heap, dijkstra_simple, a_star, bidirectional_dijkstra

st.set_page_config(
    page_title="RideSmart - Simulador Multimodal",
    page_icon="🚗",
    layout="wide"
)

# Custom style for a premium look
st.markdown("""
    <style>
    .main-title {
        font-size: 2.8rem;
        color: #1e3a8a;
        font-weight: 800;
        margin-bottom: 0.5rem;
    }
    .subtitle {
        font-size: 1.2rem;
        color: #4b5563;
        margin-bottom: 2rem;
    }
    .metric-card {
        background-color: #f3f4f6;
        border-radius: 8px;
        padding: 1.2rem;
        border-left: 5px solid #1d4ed8;
        box-shadow: 0 1px 3px rgba(0,0,0,0.1);
    }
    .metric-value {
        font-size: 1.8rem;
        font-weight: 700;
        color: #111827;
    }
    .metric-label {
        font-size: 0.9rem;
        color: #6b7280;
        text-transform: uppercase;
        font-weight: 600;
    }
    </style>
""", unsafe_allow_html=True)

# 1. Caching graph loading
@st.cache_resource
def load_data():
    ufrn_coords = (-5.8422, -35.2023)
    graph_center = (-5.835, -35.210)
    
    # Download OSM graphs
    G_osm_drive = ox.graph_from_point(graph_center, dist=2500, network_type='drive')
    G_osm_walk = ox.graph_from_point(graph_center, dist=2500, network_type='walk')
    
    G_drive = prepare_driving_graph(G_osm_drive)
    G_walk = prepare_walking_graph(G_osm_walk)
    G_drive = generate_synthetic_traffic(G_drive, congestion_center=ufrn_coords, radius=1200)
    transfer_mapping = build_transfer_mapping(G_walk, G_drive, max_transfer_distance_m=30.0)
    
    return G_drive, G_walk, transfer_mapping, G_osm_drive

with st.spinner("Carregando mapas urbanos da UFRN (OpenStreetMap)..."):
    G_drive, G_walk, transfer_mapping, G_osm_drive = load_data()

# Predefined Landmarks
LANDMARKS = {
    'Reitoria UFRN': (-5.8422, -35.2023),
    'ECT (Escola de Ciências e Tecnologia)': (-5.8437, -35.2013),
    'CT (Centro de Tecnologia)': (-5.8427, -35.210),
    'Biblioteca Central Zila Mamede': (-5.8420, -35.2005),
    'Midway Mall': (-5.8118, -35.2052),
    'Natal Shopping': (-5.8427, -35.2100),
    'Havan Natal': (-5.8166, -35.2120),
    'Coordenada Customizada': None
}

ALGORITHMS = {
    'A* (Otimizado)': a_star,
    'Dijkstra com Min-Heap': dijkstra_heap,
    'Dijkstra Bidirecional': bidirectional_dijkstra,
    'Dijkstra Simples': dijkstra_simple
}

# Sidebar inputs
st.sidebar.header("⚙️ Configurações da Rota")

# Origin Selection
origin_name = st.sidebar.selectbox("Origem (A)", list(LANDMARKS.keys()), index=2) # Default CT
if origin_name == 'Coordenada Customizada':
    o_lat = st.sidebar.number_input("Lat Origem", value=-5.8427, format="%.5f")
    o_lon = st.sidebar.number_input("Lon Origem", value=-35.2100, format="%.5f")
    origin_coords = (o_lat, o_lon)
else:
    origin_coords = LANDMARKS[origin_name]

# Destination Selection
dest_name = st.sidebar.selectbox("Destino (B)", list(LANDMARKS.keys()), index=4) # Default Midway
if dest_name == 'Coordenada Customizada':
    d_lat = st.sidebar.number_input("Lat Destino", value=-5.8118, format="%.5f")
    d_lon = st.sidebar.number_input("Lon Destino", value=-35.2052, format="%.5f")
    dest_coords = (d_lat, d_lon)
else:
    dest_coords = LANDMARKS[dest_name]

# Distance Slider
max_walk = st.sidebar.slider("Distância Máxima de Caminhada X (m)", 50, 1000, 300, step=50)

# Speed Slider
walk_speed = st.sidebar.slider("Velocidade do Pedestre (m/s)", 0.8, 2.0, 1.2, step=0.1)

# Algorithm Selection
alg_name = st.sidebar.selectbox("Algoritmo de Roteamento", list(ALGORITHMS.keys()), index=0)
selected_alg = ALGORITHMS[alg_name]

# Title
st.markdown('<div class="main-title">🚗 RideSmart: Otimização Multimodal de Rotas</div>', unsafe_allow_html=True)
st.markdown('<div class="subtitle">Simulador interativo de embarque dinâmico e integração multimodal (Pedestre + Carro).</div>', unsafe_allow_html=True)

# Run Query
if origin_coords == dest_coords:
    st.error("A origem e o destino não podem ser iguais.")
else:
    # Run optimization
    # A* needs max_speed parameter, others don't, but find_best_pickup_and_route_multimodal accepts path_algorithm
    res = find_best_pickup_and_route_multimodal(
        G_drive=G_drive,
        G_walk=G_walk,
        transfer_mapping=transfer_mapping,
        start_coords=origin_coords,
        end_coords=dest_coords,
        max_walk_distance=max_walk,
        walk_speed_mps=walk_speed,
        weight_field='time_traffic',
        path_algorithm=selected_alg
    )
    
    if not res['best_pickup_node'] or not res['drive_path']:
        st.warning("Não foi possível encontrar uma rota viável com os parâmetros fornecidos. Tente aumentar o limite de caminhada.")
    else:
        # Layout metrics
        col1, col2, col3, col4 = st.columns(4)
        
        # Calculate metric variables
        total_time_min = res['total_cost'] / 60.0
        walk_dist_m = res['walk_cost'] * walk_speed
        walk_time_min = res['walk_cost'] / 60.0
        drive_time_min = res['drive_cost'] / 60.0
        
        with col1:
            st.markdown(f"""
                <div class="metric-card">
                    <div class="metric-label">⏱️ Tempo Total</div>
                    <div class="metric-value">{total_time_min:.2f} min</div>
                    <div style="font-size: 0.85rem; color:#6b7280;">Caminhada + Carro</div>
                </div>
            """, unsafe_allow_html=True)
            
        with col2:
            st.markdown(f"""
                <div class="metric-card" style="border-left-color: #10b981;">
                    <div class="metric-label">🚶 Caminhada</div>
                    <div class="metric-value">{walk_dist_m:.1f} m</div>
                    <div style="font-size: 0.85rem; color:#10b981;">Tempo a pé: {walk_time_min:.1f} min</div>
                </div>
            """, unsafe_allow_html=True)
            
        with col3:
            st.markdown(f"""
                <div class="metric-card" style="border-left-color: #f59e0b;">
                    <div class="metric-label">🚘 Viagem de Carro</div>
                    <div class="metric-value">{drive_time_min:.2f} min</div>
                    <div style="font-size: 0.85rem; color:#6b7280;">Tempo no carro</div>
                </div>
            """, unsafe_allow_html=True)
            
        with col4:
            gain_val = res['gain']
            if gain_val is not None and gain_val > 0:
                gain_min = gain_val / 60.0
                st.markdown(f"""
                    <div class="metric-card" style="border-left-color: #10b981; background-color: #ecfdf5;">
                        <div class="metric-label" style="color: #047857;">🎉 Economia Obtida</div>
                        <div class="metric-value" style="color: #047857;">-{gain_min:.1f} min</div>
                        <div style="font-size: 0.85rem; color:#059669;">Em relação ao embarque direto</div>
                    </div>
                """, unsafe_allow_html=True)
            else:
                st.markdown("""
                    <div class="metric-card" style="border-left-color: #ef4444;">
                        <div class="metric-label">💡 Dica de Embarque</div>
                        <div class="metric-value" style="font-size: 1.4rem;">Pegar na Origem</div>
                        <div style="font-size: 0.85rem; color:#ef4444;">Caminhar não traria economia</div>
                    </div>
                """, unsafe_allow_html=True)
                
        st.write("")
        
        # Main Area: Map and Analysis Columns
        map_col, analysis_col = st.columns([7, 3])
        
        with map_col:
            st.subheader("🗺️ Mapa da Rota Multimodal Otimizada")
            
            # Draw folium map
            m = folium.Map(location=origin_coords, zoom_start=14, control_scale=True)
            
            # Highlight congestion circle
            ufrn_center = (-5.8422, -35.2023)
            folium.Circle(
                location=ufrn_center,
                radius=1200,
                color='red',
                fill=True,
                fill_color='red',
                fill_opacity=0.07,
                tooltip="Área de Congestionamento (R=1200m)"
            ).add_to(m)
            
            # Markers
            folium.Marker(location=origin_coords, popup="Origem (A)", icon=folium.Icon(color='green', icon='user', prefix='fa')).add_to(m)
            folium.Marker(location=dest_coords, popup="Destino (B)", icon=folium.Icon(color='red', icon='flag', prefix='fa')).add_to(m)
            
            # Walk route
            walk_path = res['walk_path']
            if walk_path:
                walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in walk_path]
                folium.PolyLine(walk_line, color='#1e88e5', weight=5, opacity=0.9, tooltip="Trecho de Caminhada").add_to(m)
                
            # Drive route
            drive_path = res['drive_path']
            if drive_path:
                drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in drive_path]
                folium.PolyLine(drive_line, color='#e53935', weight=4, opacity=0.8, tooltip="Trecho de Carro").add_to(m)
                
            # Pickup marker P
            p_node = res['best_pickup_node']
            p_lat, p_lon = G_drive.nodes[p_node]['y'], G_drive.nodes[p_node]['x']
            folium.Marker(
                location=(p_lat, p_lon),
                popup=f"Ponto de Embarque (P): Nó {p_node}",
                icon=folium.Icon(color='purple', icon='car', prefix='fa')
            ).add_to(m)
            
            # Display folium map in streamlit
            st_folium(m, width=900, height=500, returned_objects=[])
            
        with analysis_col:
            st.subheader("📊 Análise e Diagnóstico")
            
            # Explanatory card
            if walk_dist_m > 10.0:
                st.info(f"""
                **Análise da Decisão:**
                O algoritmo multimodal sugeriu que você caminhasse **{walk_dist_m:.1f} metros** até o ponto **Nó {p_node}**.
                
                Isso ocorre porque embarcar diretamente na origem **A** exigiria que o carro entrasse em vias muito congestionadas ou fizesse loops viários ineficientes. Caminhando um pouco, você contornou essas barreiras urbanas e economizou **{res['gain']:.1f} segundos** no total.
                """)
            else:
                st.info("""
                **Análise da Decisão:**
                O embarque ideal é na própria **Origem (A)**.
                
                Neste trajeto, a velocidade da caminhada a pé não compensaria nenhuma rota alternativa do veículo, ou a origem já se encontra fora da zona central de congestionamento.
                """)
                
            # Algorithm details
            st.write("---")
            st.subheader("⚡ Detalhes Computacionais")
            st.write(f"**Algoritmo Utilizado:** `{alg_name}`")
            
            # Show simple stats
            # We can run a small comparison using Dijkstra Heap to demonstrate the difference if not already selected
            st.markdown(f"""
            * **Nós expandidos nesta busca:** `{len(drive_path) * 4}` (estimativa)
            * **Fator de tráfego na origem:** `{G_drive.nodes[p_node].get('congestion_factor', 1.0):.2f}x`
            """)
            
            st.success("Simulação executada com sucesso na malha viária real de Natal/RN!")

st.sidebar.markdown("---")
st.sidebar.markdown("**RideSmart v1.0** — Projeto de Algoritmos II")

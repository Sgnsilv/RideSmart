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

# Configure OSMnx to use the pre-existing cache directory in the project root
ox.settings.use_cache = True
ox.settings.cache_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "cache"))

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
    '📍 Selecionar no Mapa...': None,
    'CT (Centro de Tecnologia)': (-5.8427, -35.210),
    'ECT (Escola de Ciências e Tecnologia)': (-5.8437, -35.2013),
    'Reitoria UFRN': (-5.8422, -35.2023),
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

# Initialize session state variables
if 'start_coords' not in st.session_state:
    st.session_state.start_coords = LANDMARKS['CT (Centro de Tecnologia)']
if 'dest_coords' not in st.session_state:
    st.session_state.dest_coords = LANDMARKS['Midway Mall']
if 'last_click_seen' not in st.session_state:
    st.session_state.last_click_seen = None

# We store selectbox values as raw strings in session state
if 'origin_sel_val' not in st.session_state:
    st.session_state.origin_sel_val = 'CT (Centro de Tecnologia)'
if 'dest_sel_val' not in st.session_state:
    st.session_state.dest_sel_val = 'Midway Mall'

# Map Click Event handling at the VERY TOP of the execution
if 'my_map' in st.session_state and st.session_state.my_map is not None:
    map_data = st.session_state.my_map
    if 'last_clicked' in map_data and map_data['last_clicked'] is not None:
        click = map_data['last_clicked']
        
        # Check if the click is a new event
        if st.session_state.last_click_seen != click:
            st.session_state.last_click_seen = click
            click_lat = click['lat']
            click_lon = click['lng']
            
            # Snap and update Origin
            if st.session_state.origin_sel_val == '📍 Selecionar no Mapa...':
                node = ox.distance.nearest_nodes(G_walk, X=click_lon, Y=click_lat)
                st.session_state.start_coords = (G_walk.nodes[node]['y'], G_walk.nodes[node]['x'])
                
            # Snap and update Destination
            elif st.session_state.dest_sel_val == '📍 Selecionar no Mapa...':
                node = ox.distance.nearest_nodes(G_drive, X=click_lon, Y=click_lat)
                st.session_state.dest_coords = (G_drive.nodes[node]['y'], G_drive.nodes[node]['x'])

# Sidebar inputs
st.sidebar.header("⚙️ Configurações da Rota")
st.sidebar.subheader("📍 Endereços de Referência")

# Render Origin Selectbox
origin_list = list(LANDMARKS.keys())
try:
    orig_idx = origin_list.index(st.session_state.origin_sel_val)
except ValueError:
    orig_idx = 1 # CT fallback

origin_name = st.sidebar.selectbox(
    "Origem (A)", 
    origin_list, 
    index=orig_idx,
    key="origin_widget"
)

# If selection changed, update state
if origin_name != st.session_state.origin_sel_val:
    st.session_state.origin_sel_val = origin_name
    if origin_name != '📍 Selecionar no Mapa...' and origin_name != 'Coordenada Customizada':
        st.session_state.start_coords = LANDMARKS[origin_name]
    st.rerun()

# Active warning/cancellation for Origin map selection
if origin_name == '📍 Selecionar no Mapa...':
    st.sidebar.warning("👉 Clique em qualquer ponto do mapa para definir a ORIGEM (A).")
    if st.sidebar.button("🟢 Confirmar Seleção de Origem", key="confirm_origin"):
        st.session_state.origin_sel_val = 'Coordenada Customizada'
        st.rerun()
    if st.sidebar.button("Cancelar Seleção de Origem", key="cancel_origin"):
        st.session_state.origin_sel_val = 'CT (Centro de Tecnologia)'
        st.session_state.start_coords = LANDMARKS['CT (Centro de Tecnologia)']
        st.rerun()
elif origin_name == 'Coordenada Customizada':
    o_lat = st.sidebar.number_input("Lat Origem", value=st.session_state.start_coords[0], format="%.5f")
    o_lon = st.sidebar.number_input("Lon Origem", value=st.session_state.start_coords[1], format="%.5f")
    st.session_state.start_coords = (o_lat, o_lon)

# Render Destination Selectbox
try:
    dest_idx = origin_list.index(st.session_state.dest_sel_val)
except ValueError:
    dest_idx = 5 # Midway fallback

dest_name = st.sidebar.selectbox(
    "Destino (B)", 
    origin_list, 
    index=dest_idx,
    key="dest_widget"
)

# If selection changed, update state
if dest_name != st.session_state.dest_sel_val:
    st.session_state.dest_sel_val = dest_name
    if dest_name != '📍 Selecionar no Mapa...' and dest_name != 'Coordenada Customizada':
        st.session_state.dest_coords = LANDMARKS[dest_name]
    st.rerun()

# Active warning/cancellation for Destination map selection
if dest_name == '📍 Selecionar no Mapa...':
    st.sidebar.warning("👉 Clique em qualquer ponto do mapa para definir o DESTINO (B).")
    if st.sidebar.button("🔴 Confirmar Seleção de Destino", key="confirm_dest"):
        st.session_state.dest_sel_val = 'Coordenada Customizada'
        st.rerun()
    if st.sidebar.button("Cancelar Seleção de Destino", key="cancel_dest"):
        st.session_state.dest_sel_val = 'Midway Mall'
        st.session_state.dest_coords = LANDMARKS['Midway Mall']
        st.rerun()
elif dest_name == 'Coordenada Customizada':
    d_lat = st.sidebar.number_input("Lat Destino", value=st.session_state.dest_coords[0], format="%.5f")
    d_lon = st.sidebar.number_input("Lon Destino", value=st.session_state.dest_coords[1], format="%.5f")
    st.session_state.dest_coords = (d_lat, d_lon)

# Current coordinates for optimization
origin_coords = st.session_state.start_coords
dest_coords = st.session_state.dest_coords

# Distance and Speed Sliders
max_walk = st.sidebar.slider("Distância Máxima de Caminhada X (m)", 50, 1000, 300, step=50)
walk_speed = st.sidebar.slider("Velocidade do Pedestre (m/s)", 0.8, 2.0, 1.2, step=0.1)

# Algorithm dropdown
alg_name = st.sidebar.selectbox("Algoritmo de Roteamento", list(ALGORITHMS.keys()), index=0)
selected_alg = ALGORITHMS[alg_name]

# Title
st.markdown('<div class="main-title">🚗 RideSmart: Otimização Multimodal de Rotas</div>', unsafe_allow_html=True)
st.markdown('<div class="subtitle">Simulador interativo de embarque dinâmico e integração multimodal (Pedestre + Carro).</div>', unsafe_allow_html=True)

# Run Query
if origin_coords == dest_coords:
    st.error("A origem e o destino não podem ser nas mesmas coordenadas.")
else:
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
            
            # Center map
            m = folium.Map(location=origin_coords, zoom_start=14, control_scale=True)
            
            # Congestion Circle
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
            
            # Snapping markers
            snapped_origin_node = ox.distance.nearest_nodes(G_walk, X=origin_coords[1], Y=origin_coords[0])
            snapped_origin = (G_walk.nodes[snapped_origin_node]['y'], G_walk.nodes[snapped_origin_node]['x'])
            
            snapped_dest_node = ox.distance.nearest_nodes(G_drive, X=dest_coords[1], Y=dest_coords[0])
            snapped_dest = (G_drive.nodes[snapped_dest_node]['y'], G_drive.nodes[snapped_dest_node]['x'])
            
            folium.Marker(location=snapped_origin, popup="Origem (A)", icon=folium.Icon(color='green', icon='user', prefix='fa')).add_to(m)
            folium.Marker(location=snapped_dest, popup="Destino (B)", icon=folium.Icon(color='red', icon='flag', prefix='fa')).add_to(m)
            
            # Routes
            walk_path = res['walk_path']
            if walk_path:
                walk_line = [(G_walk.nodes[n]['y'], G_walk.nodes[n]['x']) for n in walk_path]
                folium.PolyLine(walk_line, color='#1e88e5', weight=5, opacity=0.9, tooltip="Trecho de Caminhada").add_to(m)
                
            drive_path = res['drive_path']
            if drive_path:
                drive_line = [(G_drive.nodes[n]['y'], G_drive.nodes[n]['x']) for n in drive_path]
                folium.PolyLine(drive_line, color='#e53935', weight=4, opacity=0.8, tooltip="Trecho de Carro").add_to(m)
                
            p_node = res['best_pickup_node']
            p_lat, p_lon = G_drive.nodes[p_node]['y'], G_drive.nodes[p_node]['x']
            folium.Marker(
                location=(p_lat, p_lon),
                popup=f"Ponto de Embarque (P): Nó {p_node}",
                icon=folium.Icon(color='purple', icon='car', prefix='fa')
            ).add_to(m)
            
            # Display map with key="my_map" to bind its state to st.session_state.my_map
            st_folium(m, width=900, height=500, key="my_map")
            
        with analysis_col:
            st.subheader("📊 Análise e Diagnóstico")
            
            if walk_dist_m > 10.0:
                gain_val = res['gain'] if res['gain'] is not None else 0.0
                st.info(f"""
                **Análise da Decisão:**
                O algoritmo multimodal sugeriu que você caminhasse **{walk_dist_m:.1f} metros** até o ponto **Nó {p_node}**.
                
                Isso ocorre porque embarcar diretamente na origem **A** exigiria que o carro entrasse em vias muito congestionadas ou fizesse loops viários ineficientes. Caminhando um pouco, você contornou essas barreiras urbanas e economizou **{gain_val:.1f} segundos** no total.
                """)
            else:
                st.info("""
                **Análise da Decisão:**
                O embarque ideal é na própria **Origem (A)**.
                
                Neste trajeto, a velocidade da caminhada a pé não compensaria nenhuma rota alternativa do veículo, ou a origem já se encontra fora da zona central de congestionamento.
                """)
                
            # Warnings
            if origin_name == '📍 Selecionar no Mapa...':
                st.warning("⚠️ **Aguardando clique no mapa** para registrar a **Origem (A)**.")
            if dest_name == '📍 Selecionar no Mapa...':
                st.warning("⚠️ **Aguardando clique no mapa** para registrar o **Destino (B)**.")
                
            st.success("""
            **💡 Como selecionar no mapa:**
            1. Selecione `📍 Selecionar no Mapa...` no menu de Origem ou Destino na barra lateral.
            2. Clique em qualquer rua do mapa.
            3. A coordenada será **snappada automaticamente à via válida mais próxima**!
            """)
            
            st.write("---")
            st.subheader("⚡ Detalhes Computacionais")
            st.write(f"**Algoritmo Utilizado:** `{alg_name}`")
            
            st.markdown(f"""
            * **Origem atual:** `({origin_coords[0]:.5f}, {origin_coords[1]:.5f})`
            * **Destino atual:** `({dest_coords[0]:.5f}, {dest_coords[1]:.5f})`
            * **Fator de tráfego na origem:** `{G_drive.nodes[p_node].get('congestion_factor', 1.0):.2f}x`
            """)

st.sidebar.markdown("---")
st.sidebar.markdown("**RideSmart v1.0** — Projeto de Algoritmos II")

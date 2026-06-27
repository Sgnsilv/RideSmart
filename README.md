# RideSmart
**Modelagem e Análise de Rotas Urbanas com Grafos**

Repositório destinado ao projeto final (Unidade III) da disciplina de **Algoritmos e Estruturas de Dados II (DCA0209)** do Departamento de Engenharia de Computação e Automação (DCA) da **Universidade Federal do Rio Grande do Norte (UFRN)**.

---

## Sobre o Projeto
O **RideSmart** simula um problema inspirado em aplicativos reais de mobilidade urbana (como Uber, 99 ou Waze). O objetivo é encontrar o equilíbrio ideal (*trade-off*) entre o tempo de caminhada de um usuário e a rota de um veículo de aplicativo, otimizando o ponto de embarque. 

Dado um ponto de origem $A$, um destino $B$ e uma distância máxima de caminhada $X$ metros, o sistema analisa a malha urbana viária de Natal/RN para encontrar o melhor ponto de embarque $P$, formando a rota:
* $A \to P$ (Caminhada)
* $P \to B$ (Carro)

---

## Tecnologias Utilizadas
* **Python 3**
* **Jupyter Notebook**
* **OSMnx** (Manipulação e download de redes viárias reais do OpenStreetMap)
* **NetworkX** (Estrutura de dados e utilitários de grafos)
* **Folium** (Visualização cartográfica interativa)
* **Matplotlib & Pandas** (Plotagem e consolidação das métricas experimentais)

---

## Modelagem do Problema

### 1. Representação do Grafo
* **Nós ($V$)**: Representam os cruzamentos e bifurcações da malha viária urbana (região de Lagoa Nova e Campus Central da UFRN). Contêm atributos geográficos de latitude e longitude.
* **Arestas ($E$)**: Representam os segmentos de vias de sentido único ou duplo. Possuem atributos de distância física (`length`), velocidade máxima (`maxspeed`) e tempos de viagem.

### 2. Funções de Peso (Custos)
* **Distância (`length`)**: Comprimento físico da via em metros.
* **Tempo em Fluxo Livre (`time_free_flow`)**: Tempo ideal de tráfego de carro calculado como:
  $$T_{free\_flow} = \frac{\text{comprimento (m)}}{\text{velocidade máxima (m/s)}}$$
* **Tempo com Trânsito Sintético (`time_traffic`)**: Adiciona um fator multiplicativo basal aleatório ($1.0\text{ a }1.25\times$) e uma penalidade concêntrica (de até $3.25\times$) centrada na UFRN para simular congestionamentos em horários de pico.

### 3. Busca Multimodal
Para uma dada origem $A$ e restrição de caminhada $X$:
1. Buscamos todas as interseções $P$ alcançáveis a pé com distância $D_{walk}(A \to P) \le X$.
2. Para cada $P$, calculamos a rota ótima do veículo de $P \to B$.
3. O ponto $P$ que minimiza o tempo total ($T_{walk} + T_{drive}$) é escolhido.
4. O resultado é comparado com o caso sem caminhada ($P = A$) para medir a economia real obtida.

---

## Como Executar o Projeto

### 1. Instalar as Dependências
Recomenda-se a criação de um ambiente virtual Python para isolar as dependências do projeto:

```bash
# Criar ambiente virtual
python3 -m venv .venv

# Ativar ambiente virtual
source .venv/bin/activate

# Instalar pacotes requeridos
pip install osmnx networkx matplotlib pandas folium ipykernel
```

### 2. Executar o Notebook Jupyter
Inicie o Jupyter Notebook ou abra o arquivo diretamente em sua IDE (como VS Code):

```bash
jupyter notebook RideSmart_Notebook.ipynb
```

Execute as células em sequência para ver a simulação passo a passo, a plotagem dos mapas interativos do Folium e a execução dos benchmarks de tempo.

---

## 📊 Algoritmos Comparados

Todos os algoritmos foram implementados manualmente a partir do zero no arquivo [algorithms.py](file:///home/gabshys/Documentos/UFRN/7P/RideSmart/RideSmart/algorithms.py) para fins de comparação didática:

1. **Dijkstra Clássico (Simples)**: Busca linear do nó de custo mínimo no conjunto de abertos. Complexidade $O(V^2)$.
2. **Dijkstra com Fila de Prioridade (Heap)**: Utiliza min-heap para extrair o nó mínimo de forma eficiente. Complexidade $O((V+E) \log V)$.
3. **A\***: Otimizado com heurística geográfica de Haversine admissível (distância dividida pela velocidade máxima viária).
4. **Dijkstra Bidirecional** (Algoritmo da literatura): Duas buscas simultâneas que se encontram no meio do caminho. Reduz drasticamente o espaço de busca na prática para consultas ponto a ponto.

---

## 📈 Resultados do Benchmark
Abaixo está o resumo médio obtido a partir de 50 consultas de rotas aleatórias no grafo de Natal/RN:

| Algoritmo | Tempo Médio (ms) | Nós Expandidos Médios | Eficiência de Busca |
| :--- | :---: | :---: | :---: |
| **Dijkstra Simples** | ~2.50 ms | 134.5 | Ruim ($O(V^2)$) |
| **Dijkstra Heap** | ~0.22 ms | 135.5 | Excelente |
| **A\*** | ~0.35 ms | 98.4 | Altamente Direcionada |
| **Dijkstra Bidirecional** | ~0.26 ms | 88.2 | Menor Espaço de Busca |

*Nota: O Dijkstra Bidirecional e o A\* reduzem significativamente o número de nós expandidos durante a busca. O Dijkstra com Heap é ordens de magnitude mais rápido que a implementação Simples ($O(V^2)$).*

---

## 👥 Equipe
* **ICARO BRUNO SILBE CORTÊS**  
* **GABRIEL SEBASTIAO DO NASCIMENTO NETO**  
* **SARA GABRIELLY DO NASCIMENTO SILVA**

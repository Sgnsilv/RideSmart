# RideSmart
**Modelagem e Análise de Rotas Urbanas com Grafos**

Repositório destinado ao projeto final (Unidade III) da disciplina de **Algoritmos e Estruturas de Dados II (DCA0209)**. 

## 📌 Sobre o Projeto
O **RideSmart** simula um problema inspirado em aplicativos reais de mobilidade urbana. O objetivo é encontrar o equilíbrio ideal (trade-off) entre o tempo de caminhada de um usuário e a rota de um veículo, otimizando o ponto de embarque. 

Dado um ponto de origem `A`, um destino `B` e uma distância máxima de caminhada `X`, o sistema avalia a malha urbana para escolher o melhor ponto de embarque `P`, formando a rota:
* `A → P` (Caminhada)
* `P → B` (Carro)

## 🎯 Objetivos e Algoritmos Avaliados
O projeto implementa e compara o desempenho de diferentes algoritmos de caminhos mínimos aplicados a dados reais extraídos do OpenStreetMap. Foram analisados:
1. **Dijkstra Clássico**
2. **Dijkstra com Fila de Prioridade (Heap)**
3. **A*** (com heurística geográfica)

Os testes incluem cenários de menor distância, rotas mais rápidas sem trânsito e rotas impactadas por trânsito sintético.

## 🛠️ Tecnologias Utilizadas
* **Python 3**
* **Jupyter Notebook**
* **OSMnx** (Manipulação de dados geográficos)
* **NetworkX** (Estrutura e análise de grafos)

## 👥 Equipe
* 
* ICARO BRUNO SILBE CORTÊS  
* GABRIEL SEBASTIAO DO NASCIMENTO NETO
* SARA GABRIELLY DO NASCIMENTO SILVA

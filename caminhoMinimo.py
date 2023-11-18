import sys
import time
import heapq
import random
import networkx as nx
import matplotlib.pyplot as plt

class Grafo:
    def __init__(self, numVertices, origem=0, destino=None):
        self.numVertices = numVertices
        self.origem = origem
        self.destino = destino if destino else numVertices - 1
        self.matAdj = [[sys.maxsize for _ in range(self.numVertices)] for _ in range(self.numVertices)]

    @property
    def numVertices(self):
        return self._numVertices

    @numVertices.setter
    def numVertices(self, numVertices):
        self._numVertices = numVertices

    @property
    def matAdj(self):
        return self._matAdj

    @matAdj.setter
    def matAdj(self, matAdj):
        self._matAdj = matAdj

    def adicionarAresta(self, verticeA, verticeB, peso):
        self._matAdj[verticeA][verticeB] = peso
        # self._matAdj[verticeB][verticeA] = peso
    
    def removerAresta(self, verticeA, verticeB):
        self._matAdj[verticeA][verticeB] = sys.maxsize

    def dijkstra(self):
        distancias = [sys.maxsize for _ in range(self.numVertices)]
        antecessores = [sys.maxsize for _ in range(self.numVertices)]
        distancias[self.origem] = 0
        antecessores[self.origem] = self.origem
        heap = [(0, self.origem)]
        while heap:
            _, verticeAtual = heapq.heappop(heap)
            for j in range(self.numVertices):
                if self._matAdj[verticeAtual][j] != sys.maxsize:
                    novaDistancia = distancias[verticeAtual] + self._matAdj[verticeAtual][j]
                    if novaDistancia < distancias[j]:
                        antecessores[j] = verticeAtual
                        distancias[j] = novaDistancia
                        heapq.heappush(heap, (novaDistancia, j))
        return distancias, antecessores

    def encontraCaminhoMinimo(self):
        distancias, antecessores = self.dijkstra()
        caminho = []
        if distancias[self.destino] == sys.maxsize:
            return caminho
        atual = self.destino
        while atual != self.origem:
            caminho.append(atual)
            atual = antecessores[atual]
        caminho.reverse()
        return caminho

    def mostraGrafo(self):
        print("    ", end="")
        for i in range(self.numVertices):
            print(f" {i}: ", end="")
        print("")
        for i in range(self.numVertices):
            print(f" {i}: ", end="")
            for j in range(self.numVertices):
                if (self.matAdj[i][j] == sys.maxsize):
                    print(f"{0:>3}", end=" ")
                else:
                    print(f"{self.matAdj[i][j]:>3}", end=" ")
            print("")

    def plotarGrafo(self, nome_grafo, mip_edges):
        G = nx.DiGraph()

        for i in range(self.numVertices):
            G.add_node(i)

        for i in range(self.numVertices):
            for j in range(self.numVertices):
                if self.matAdj[i][j] != sys.maxsize:
                    G.add_edge(i, j, weight=self.matAdj[i][j])

        pos = nx.circular_layout(G)  # You can use a different layout if needed
        labels = nx.get_edge_attributes(G, 'weight')
        nx.draw(G, pos, with_labels=True)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

        # experimental
        path = nx.shortest_path(G,source=self.origem,target=self.destino)
        path_edges = list(zip(path,path[1:]))
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='r')
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='r', alpha=0.5, width=8)
        nx.draw_networkx_edges(G, pos, edgelist=mip_edges, edge_color='g', alpha=0.8, width=4)
        
        plt.savefig(nome_grafo)
        plt.show()

def ler_arquivo_grafo(nome_arquivo, numero_vertices):
    grafo = Grafo(numero_vertices)
    with open(nome_arquivo, 'r') as arquivo:
        for linha in arquivo:
            dados = linha.strip().split(',')
            vertice1 = int(dados[0])
            vertice2 = int(dados[1])
            peso = int(dados[2])
            grafo.adicionarAresta(vertice1, vertice2, peso)
    return grafo

from mip import Model, xsum, minimize, BINARY, OptimizationStatus

def shortest_path_mip(grafo, origem, destino):
    modelo = Model()

    # variáveis de decisão
    x = [[modelo.add_var(var_type=BINARY) for j in range(len(grafo))] for i in range(len(grafo))]

    # função objetivo
    modelo.objective = minimize(xsum(grafo[i][j] * x[i][j] for i in range(len(grafo)) for j in range(len(grafo))))

    # restrições
    # modelo += xsum(x[origem][j] for j in range(len(grafo))) == 1  # restrição de origem
    # for k in range(len(grafo)):
    #     modelo += xsum(x[i][k] for i in range(len(grafo))) - xsum(x[k][j] for j in range(len(grafo))) == 0  # restrição de fluxo
    # modelo += -xsum(x[i][destino] for i in range(len(grafo))) == -1  # restrição de destino

    # is the following correct?
    modelo += xsum(x[origem][j] for j in range(len(grafo))) == 1  # restrição de origem
    for k in range(origem+1, destino):
        modelo += xsum(x[i][k] for i in range(len(grafo))) + xsum(-1 * x[k][j] for j in range(len(grafo))) == 0  # restrição de fluxo
    modelo += xsum(x[i][destino] for i in range(len(grafo))) == 1  # restrição de destino

    # encontrando caminho mínimo
    modelo.optimize()

    return [(i, j) for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99]
    # return [i for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99]  # retornando resultado

# gera um grafo aleatório com base no número de vértices e arestas
def gerar_grafo_aleatorio(num_vertices, num_arestas, semente=42):
    G = nx.gnm_random_graph(num_vertices, num_arestas, directed=True, seed=semente)
    # G = nx.dense_gnm_random_graph(num_vertices, num_arestas, seed=semente)
    # G = nx.gnp_random_graph(num_vertices, 0.25, seed=semente, directed=True)
    for aresta in G.edges:
        peso = random.randint(5, 100)
        G.edges[aresta]['weight'] = peso
    return G

# função que permite visualizar e salvar imagem dos grafos gerados
def visualizar_grafo(G, nome_grafo):
    pos = nx.spring_layout(G)
    rotulos = nx.get_edge_attributes(G, 'weight')
    nx.draw(G, pos, with_labels=True, font_weight='bold', arrowsize=15)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=rotulos)
    plt.savefig(nome_grafo)
    plt.show()

def instanciar_grafo(grafo_aleatorio):
    grafo = Grafo(len(grafo_aleatorio.nodes))
    for aresta in grafo_aleatorio.edges:
        origem, destino = aresta
        peso = grafo_aleatorio.edges[aresta]['weight']
        grafo.adicionarAresta(origem, destino, peso)
    return grafo

grafos = []        # lista para armazenar os grafos gerados
n = 21              # número inicial de vértices
mip_results = []
idx = 0

for i in range(5):
    # utilitários
    origem = 0
    destino = n - 1
    n_arestas = 2 * (n - 1)

    # gerando, salvando a imagem, instanciando e adicionando o grafo aleatório à lista de grafos
    grafo_aleat = gerar_grafo_aleatorio(n, n_arestas, semente=43)
    grafo_aleat = instanciar_grafo(grafo_aleat)
    grafos.append(grafo_aleat)

    # encontrando caminho mínimo
    resultado_dijkstra = [grafo_aleat.origem] + grafo_aleat.encontraCaminhoMinimo()
    resultado_MIP = shortest_path_mip(grafo_aleat._matAdj, grafo_aleat.origem, grafo_aleat.destino)
    # print(resultado_MIP)
    mip_results.append(resultado_MIP)
    print(f"\nCaminho minimo PARTINDO de ({grafo_aleat.origem}) PARA ({grafo_aleat.destino}):\nDIJKSTRA: {resultado_dijkstra}\nMIP: {resultado_MIP}\n")

    # dobrando o número de vértices e o número de arestas para o próximo grafo
    n += 1
    # num_arestas += 4

grafos[idx].plotarGrafo(f'grafo{idx+1}.svg', mip_results[idx])
grafos[idx].mostraGrafo()
print(f'MIP RESULTS: {mip_results[idx]}')
# cont = 1
# for grafo in grafos:
#     grafo.plotarGrafo(f'grafo{cont}.png')
#     grafo.mostraGrafo()
#     # time.sleep(2)
#     cont += 1

# visualizar_grafo(grafos[0], f'grafo1.png')
# visualizar_grafo(grafos[1], f'grafo2.png')
# visualizar_grafo(grafos[2], f'grafo3.png')
# visualizar_grafo(grafos[3], f'grafo4.png')
# visualizar_grafo(grafos[4], f'grafo5.png')



# teste_grafo = instanciar_grafo(random_integer_weighted_directed_graph)
cam_min1 = grafos[idx].encontraCaminhoMinimo()
print(f'DIJKSTRA: {[grafos[idx].origem] + cam_min1}')
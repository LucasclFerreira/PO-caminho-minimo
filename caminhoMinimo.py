import sys
import heapq
import random
import networkx as nx
import matplotlib.pyplot as plt
from mip import Model, xsum, minimize, BINARY

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

        pos = nx.shell_layout(G)  # You can use a different layout if needed
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

# Caminho mínimo usando MIP
def shortest_path_mip(grafo, origem, destino):
    modelo = Model()

    # variáveis de decisão
    x = [[modelo.add_var(var_type=BINARY) for j in range(len(grafo))] for i in range(len(grafo))]

    # função objetivo
    modelo.objective = minimize(xsum(grafo[i][j] * x[i][j] for i in range(len(grafo)) for j in range(len(grafo))))

    # restrições
    modelo += xsum(x[origem][j] for j in range(len(grafo))) == 1  # restrição de origem
    for k in range(1, len(grafo) - 1):
        if k != destino and k != origem:
            modelo += xsum(x[i][k] for i in range(len(grafo))) - xsum(x[k][j] for j in range(len(grafo))) == 0  # restrição de fluxo
    modelo += -xsum(x[i][destino] for i in range(len(grafo))) == -1  # restrição de destino

    # encontrando caminho mínimo
    modelo.optimize()

    # retornando uma tupla com a solução no seguinte formato: (arestas, vértices)
    return ([(i, j) for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99], [i for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99])


# gera um grafo aleatório com base no número de vértices e arestas
def gerar_grafo_aleatorio(num_vertices, num_arestas, semente=42):
    G = nx.gnm_random_graph(num_vertices, num_arestas, directed=True, seed=semente)
    for aresta in G.edges:
        peso = random.randint(5, 100)
        G.edges[aresta]['weight'] = peso
    return G

# instancia da classe Grafo a partir de um grafo aleatório
def instanciar_grafo(grafo_aleatorio, destino):
    grafo = Grafo(len(grafo_aleatorio.nodes), destino=destino)
    for aresta in grafo_aleatorio.edges:
        origem, destino = aresta
        peso = grafo_aleatorio.edges[aresta]['weight']
        grafo.adicionarAresta(origem, destino, peso)
    return grafo

# variáveis utilitárias
n_vertices = 23
n_arestas = 2 * (n_vertices - 1)
destino = 10

# criando grafo
novo_grafo = gerar_grafo_aleatorio(n_vertices, n_arestas, semente=39)
novo_grafo = instanciar_grafo(novo_grafo, destino=destino)

# resultados
resultado_dijkstra = [novo_grafo.origem] + novo_grafo.encontraCaminhoMinimo()
arestas_MIP, resultado_MIP = shortest_path_mip(novo_grafo._matAdj, novo_grafo.origem, novo_grafo.destino)
novo_grafo.plotarGrafo(f'grafo.svg', arestas_MIP)

print(f"\nCaminho minimo PARTINDO de ({novo_grafo.origem}) PARA ({novo_grafo.destino}):\nDIJKSTRA: {resultado_dijkstra}\nMIP: {resultado_MIP + [destino]}\n")
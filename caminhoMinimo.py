

import sys
import heapq
import networkx as nx
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.colors as colors

class Grafo:
    def __init__(self, numVertices):
        self.numVertices = numVertices
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

    def dijkstra(self, origem):
        distancias = [sys.maxsize for _ in range(self.numVertices)]
        antecessores = [sys.maxsize for _ in range(self.numVertices)]
        distancias[origem] = 0
        antecessores[origem] = origem
        heap = [(0, origem)]
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

    def encontraCaminhoMinimo(self, origem, destino):
        distancias, antecessores = self.dijkstra(origem)
        caminho = []
        if distancias[destino] == sys.maxsize:
            return caminho
        atual = destino
        while atual != origem:
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

# altere o numero de vertices para o codigo não quebrar
grafo = ler_arquivo_grafo('grafo1.txt', 6)

# teste para ver se caminho minimo funciona
origem = 0
destino = 5
caminhoMinimo = [origem] + grafo.encontraCaminhoMinimo(origem, destino)
print(f"\nCaminho minimo PARTINDO de ({origem}) PARA ({destino}): {caminhoMinimo}\n")

grafo.mostraGrafo()


# MODELO LINEAR ABAIXO AINDA SEM TESTE
# from mip import Model, xsum, minimize, BINARY

# def shortest_path_mip(grafo, source, destination):
#     # Initialize the model
#     model = Model()

#     # Define the variables
#     x = [[model.add_var(var_type=BINARY) for j in range(len(grafo))] for i in range(len(grafo))]

#     # Define the objective function
#     model.objective = minimize(xsum(grafo[i][j]*x[i][j] for i in range(len(grafo)) for j in range(len(grafo))))

#     # Define the constraints
#     for i in range(len(grafo)):
#         model += xsum(x[i][j] for j in range(len(grafo))) == 1
#         model += xsum(x[j][i] for j in range(len(grafo))) == 1

#     # Additional constraints for source and destination
#     model += xsum(x[source][j] for j in range(len(grafo))) == 1
#     model += xsum(x[j][destination] for j in range(len(grafo))) == 0

#     # Solve the model
#     model.optimize()

#     # Return the solution
#     return [[i, j] for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99]
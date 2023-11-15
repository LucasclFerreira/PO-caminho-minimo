import sys
import heapq

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

from mip import Model, xsum, minimize, BINARY

def shortest_path_mip(grafo, origem, destino):
    model = Model()

    # variáveis de decisão
    x = [[model.add_var(var_type=BINARY) for j in range(len(grafo))] for i in range(len(grafo))]

    # função objetivo
    model.objective = minimize(xsum(grafo[i][j] * x[i][j] for i in range(len(grafo)) for j in range(len(grafo))))

    # restrições
    model += xsum(x[origem][j] for j in range(len(grafo))) == 1  # restrição de origem
    for i in range(len(grafo)):
        model += xsum(x[j][i] for j in range(len(grafo))) - xsum(x[i][j] for j in range(len(grafo))) == 0  # restrição de fluxo
    model += xsum(x[j][destino] for j in range(len(grafo))) == 1  # restrição de destino

    # encontrando caminho mínimo
    model.optimize()

    return [i for i in range(len(grafo)) for j in range(len(grafo)) if x[i][j].x >= 0.99]  # retornando resultado

if len(sys.argv) < 5:
    print("Usage: python caminhoMinimo.py <nome_arquivo> <quantidade_vertices> <origem> <destino>")
    sys.exit(1)

nome_arquivo = sys.argv[1]
quantidade_vertices = int(sys.argv[2])
origem = int(sys.argv[3])
destino = int(sys.argv[4])

grafo = ler_arquivo_grafo(nome_arquivo, quantidade_vertices)
grafo.mostraGrafo()

resultado_dijkstra = [origem] + grafo.encontraCaminhoMinimo(origem, destino)
resultado_MIP = shortest_path_mip(grafo._matAdj, origem, destino)

print(f"\nCaminho minimo PARTINDO de ({origem}) PARA ({destino}):\nDIJKSTRA: {resultado_dijkstra}\nMIP: {resultado_MIP}\n")
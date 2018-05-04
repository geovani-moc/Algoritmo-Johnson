/**  ________________________________________________
    |    PROGRAMADOR: GEOVANI PEREIRA DOS SANTOS     |
    |       IFNMG - MONTES CLAROS - MG - 2017        |
    |________________________________________________|
**/
#include <iostream>
#include <stdio.h>
#include <vector>
#include <queue>

using namespace std;
enum cores{branco, cinza, preto};
#define INF 100000000

class aresta_adj
{
private:
    int vert, viz, custo;
public:
    aresta_adj():vert(0), viz(0), custo(0){}
    aresta_adj(int vert, int viz, int custo):vert(vert), viz(viz), custo(custo){}
    bool operator< (const int a)const;
    bool operator< (const aresta_adj & a)const;
    int get_custo(){return custo;}
    void set_custo(int custo){this->custo = custo;}
    int get_viz(){return viz;}
    int get_vert(){return vert;}

    friend class grafo;
};
class grafo //abstraçao de um grafo
{
private:
    vector <vector <aresta_adj> > adj;
    int vertices, arestas, tam;
public:
    grafo();
    grafo(int v, int a);
    void inserir(int a, int b, int custo);
    grafo& operator= (const grafo & G);
    friend bool bellman_ford(grafo& g2, vector<int> & custo_min);
    friend void jhonson(grafo & G);
    friend void potencial(vector<int> & custo, grafo & g);
    friend void dijkstra(grafo & G, int origem, int destino, const vector<int> & custo);
    friend int busca_peso_aresta(int v, int w, grafo &g);
};
bool bellman_ford(grafo & g2, vector<int> & custo_min);
bool relaxa(vector<int> & custo, int v , int w, int peso);
void potencial(vector<int> & custo, grafo & g);
void dijkstra(grafo & G, int origem, int destino, const vector<int> & custo);
int busca_peso_aresta(int v, int w, grafo &g);
void jhonson(grafo & G);
/// ____________________MAIN __________________________________________________
int main()
{
    int vertices, arestas, a, b, custo;
    scanf("%i %i",&vertices, &arestas);
    grafo G(vertices, arestas);
    for(int i=0; i < arestas; i++)
    {
        scanf("%i %i %i",&a,&b,&custo);//leitura dos vertices do grafo
        G.inserir(a, b, custo);
    }
    jhonson(G);
    return 0;
}
/// ___________________CLASSE ARESTA_ADJ ______________________________________
bool aresta_adj::operator<(const int a)const
{
    if(custo < a)return true;
    return false;
}
bool aresta_adj::operator<(const aresta_adj & a)const
{
    if(custo > a.custo)return true;
    return false;
}
/// ___________________CLASSE GRAFO ___________________________________________
grafo::grafo()
{
    adj.clear();
    vertices = arestas = tam =0;
}
grafo::grafo(int v, int a)
{
    vertices = tam = v;
    arestas = a;
    adj.clear();
    adj.resize(v);
}
void grafo::inserir(int a, int b, int custo)
{
    adj[a].push_back(aresta_adj(a, b, custo));
}
grafo& grafo::operator=(const grafo & G)
{
    if(this != &G)
    {
        if(G.adj.size() != adj.size())
        {
            adj.clear();
            adj.resize(G.adj.size());
        }
        adj = G.adj;
        vertices = G.vertices;
        arestas = G.arestas;
        tam = G.tam;
    }
    return *this;
}

/// ___________________FUNÇÃO BELLMAN-FORD ____________________________________
bool bellman_ford(grafo &g2, vector<int> &custo_min)//relaxamento de arestas, para o calculo de potenciais, e verificaçao de existencia de ciclo negativo
{
    int w, peso;
    for(int i=0; i < g2.tam-1; i++)
        for(int v=0; v < g2.tam; v++)
            for(unsigned int j=0; j < g2.adj[v].size(); j++)
            {
                w = g2.adj[v][j].get_viz();
                peso = g2.adj[v][j].get_custo();
                relaxa(custo_min, v, w, peso);
            }
    for(int v=0; v < g2.tam; v++)//verifica se existe ciclo
        for(unsigned int j=0; j < g2.adj[v].size(); j++)
        {
            w = g2.adj[v][j].get_viz();
            peso = g2.adj[v][j].get_custo();
            if(relaxa(custo_min, v, w, peso))
                return false;
        }
    return true;
}

bool relaxa( vector<int> & custo, int v , int w, int peso)//relaxamento de aresta
{
    if(custo[w] > (custo[v] + peso))
    {
        custo[w] = custo[v]+peso;
        return true;
    }
    return false;
}
/// ___________________FUNÇÃO DIJKSTRA_________________________________________
void dijkstra(grafo & G, int origem, int destino, const vector<int> & custo)//busca o menor camindo de  v para w, se existir, e mostra a distancia
{
    int dist[G.tam], pai[G.tam], visitados[G.tam];
    priority_queue <aresta_adj> p;
    aresta_adj aux;
    for(int i=0; i<G.tam; i++)//inicializaçao de componentes
    {
        pai[i] = -1;
        visitados[i] = branco;
        dist[i] = INF;
    }
    for(unsigned int i=0; i<G.adj[origem].size(); i++)//pre processamento do vertice origem
    {
        p.push(G.adj[origem][i]);
        visitados[( G.adj[origem][i].get_viz() )] = cinza;
    }
    dist[origem] = 0;
    pai[origem] = origem;
    visitados[origem] = preto;

    while(!p.empty())
    {
        aux = p.top();
        p.pop();
        visitados[aux.get_vert()] = preto;
        if(visitados[destino] == preto)
            break;
        for(unsigned int i=0; i<G.adj[aux.get_viz()].size(); i++)
        {
            if( (dist[aux.get_vert()] + aux.get_custo() ) < dist[aux.get_viz()] ) //verifica necessidade de relaxar aresta
            {
                dist[aux.get_viz()] = dist[aux.get_vert()] + aux.get_custo();//relaxa aresta
                pai[aux.get_vert()] = aux.get_viz();//o valor de indece indica quem e o pai , e o valor nessa possiçao de indece o filho
            }
            if(visitados[G.adj[aux.get_viz()][i].get_vert()] != preto)//adiciona arestas vizinhas na lista de prioridade
            {
                p.push(G.adj[aux.get_viz()][i]);
                visitados[aux.get_viz()] = cinza;
            }
        }
        if(G.adj[aux.get_viz()].size() == 0)//processamento do caso especial onde um vertice não tem arestas saindo para outros vertices
        {
            visitados[aux.get_viz()] = preto;
            pai[aux.get_vert()] = aux.get_viz();
        }

    }
    if(visitados[destino] == branco)
        cout << "Caminho inexistente" << endl;
    else
    {
        int result = 0, temp = origem;
        while(temp != destino)//calcula a distancia de v para w
        {
            result += ( busca_peso_aresta(temp, pai[temp], G) - custo[temp] )+ custo[ pai[temp] ];
            temp = pai[temp];
        }
        cout << result << endl;
    }
}
int busca_peso_aresta(int v, int w, grafo &g)
{
    for(unsigned int i=0; i<g.adj[v].size(); i++)
        if(g.adj[v][i].get_viz() == w)
            return g.adj[v][i].get_custo();
    return 0;
}
/// ___________________FUNÇÃO JHONSON _________________________________________
void jhonson(grafo & G)// executa o bellman-ford , calcula os potenciais, executa o dijkstra e mostra o menor caminho
{
    grafo g2 = G;
    vector<int> custo_min(g2.arestas,0);
    if(!bellman_ford(g2,custo_min))
    {
        cout << "Contem ciclo negativo" ;
        return;
    }
    potencial(custo_min, g2);
    int quantidade_buscas, v, w;
    scanf("%i", &quantidade_buscas);
    for(int i=0; i<quantidade_buscas; i++)
    {
        scanf("%i %i", &v, &w);
        dijkstra(g2, v, w, custo_min);
    }
}
void potencial(vector<int> & custo, grafo & g)//calcula os potenciais
{
    int custo_atual, w;
    for(int i=0; i < g.tam; i++)
        for(unsigned int j=0; j<g.adj[i].size(); j++)
        {
            custo_atual = g.adj[i][j].get_custo();
            w = g.adj[i][j].get_viz();

            g.adj[i][j].set_custo( ( (custo_atual + custo[i]) - custo[w]) );
        }
}

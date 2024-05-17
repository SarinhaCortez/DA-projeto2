#include "Utils.h"

using namespace std;

using namespace std;


bool BacktrakingTSP(MGraph &g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited){

    bool final_found = false;
    if(vertex==0 && numVertex==0 && visited.find(vertex)!=visited.end()){

        if(dist<minDist){
            minDist=dist;
            final.clear();
            for(auto element: tsp){
                final.push_back(element);
            }


            found=true;
            return found;
        }

        return false;
    }

    else if((numVertex>0 && vertex!=0) || (visited.find(vertex)==visited.end() && vertex==0 && numVertex>0)) {

        for (auto dest: g.getAdj(vertex)) {

            if ((visited.find(dest)==visited.end() && dest!=0) || (visited.find(dest)==visited.end() && dest==0 && numVertex==1)) {
                visited.insert(dest);
                tsp.push_back(dest);

                int newNumVertex=numVertex - 1;
                int newDist=dist + g.getWeight(vertex, dest);

                if (BacktrakingTSP(g, newDist, minDist, tsp, final, newNumVertex, dest, found, visited)){

                    visited.erase(dest);
                    tsp.pop_back();
                    final_found=true;
                    /*dist = newDist-edge->getWeight();
                    numVertex++;*/
                }
                else{

                    visited.erase(dest);
                    tsp.pop_back();
                    /*dist = newDist-edge->getWeight();
                    numVertex++;*/
                    return found;}
            }
        }
    }
    else{

        return false;
    }
    visited.erase(vertex);

    return final_found;
}

//Nearest neighbour strategy

bool notPresent(int i, std::vector<int> &tsp){
    for(auto el: tsp){
        if(i == el){
            return false;
        }
    }
    return true;
}

pair<int, int> minDist(std::vector<double> currentRow, std::vector<int> &tsp){
    int min=INT_MAX;
    int minVertex;
    pair<int, int> res;

//se o indicw do minimo tiver no tsp nao da

    for(int i=0; i<currentRow.size(); i++){
        //cout << currentRow[i] <<"---->" << i <<endl;
        if(currentRow[i] >=0 && currentRow[i] <min && notPresent(i, tsp)){/* nao esta em tsp*/
            //cout <<"ENTREI"<<endl;
            min = currentRow[i];
            minVertex=i;
            //cout<<min<<"---->aa"<<minVertex<<endl;
        }
    }

    res.first = min;
    res.second=minVertex;

    return res;
}

bool nearestNeighbour(MGraph &g,  std::vector<int> &tsp, std::vector<std::vector<double>>* matrix){
    int initialVertex = 0;
    std::vector<double> currentRow;
    int count=0;
    int totalDist=0;

    while(count<matrix->size()-1){
        currentRow = (*matrix)[initialVertex];

        pair<int, int> minDistance = minDist(currentRow, tsp);
        tsp.push_back(minDistance.second);
        totalDist+=minDistance.first;

        if(minDistance.first == INT_MAX){
            cout <<"nao existe lol"<<endl;
            return false;
        }

        //cout<< "minDist: "<<minDistance.first <<" ->> totalDist: "<<totalDist<<endl;

        count++;
        initialVertex=minDistance.second;
    }

    tsp.push_back(0);
    totalDist+=(*matrix)[0][initialVertex];
    for(auto v: tsp){
        cout<<v<<endl;
    }
    cout<<"Total min Dist: "<<totalDist<<endl;
    return true;


}



/*
double distanceNodes(pair<int, std::pair<double, double>>& point1, pair<int, std::pair<double, double>>& point2) {
    double sum = 0.0;
            double diff1 = point1.second.first - point2.second.first;
        double diff2 = point1.second.second - point2.second.second;
        sum += diff1 * diff1;
    sum += diff2 * diff2;

    return sqrt(sum);
}


void assignClusters(std::vector<pair<int, std::pair<double, double>>>& data,  vector<pair<int, std::pair<double, double>>>& centroids, vector<pair<int, int>>& labels) {
    for (size_t i = 0; i < data.size(); ++i) {
        double minDistance = numeric_limits<double>::max();
        int cluster = -1;
        for (size_t j = 0; j < centroids.size(); ++j) {
            double d = distanceNodes(data[i], centroids[j]);
            if (d < minDistance) {
                minDistance = d;
                cluster = j;
            }
        }
        labels[i].second = cluster;
        labels[i].first = data[i].first;
    }
}

void updateCentroids(vector<pair<int, std::pair<double, double>>>& data, vector<pair<int, std::pair<double, double>>>& centroids, vector<pair<int, int>>& labels) {

    vector<vector<double>> sums(centroids.size(), vector<double>(2, 0.0));
    vector<int> counts(centroids.size(), 0);

    for (size_t i = 0; i < data.size(); ++i) {
        int cluster = labels[i].second;

            sums[cluster][0] += data[i].second.first;
            sums[cluster][1] += data[i].second.second;

        counts[cluster]++;
    }

    for (size_t i = 0; i < centroids.size(); ++i) {

            centroids[i].second.first = sums[i][0] / counts[i];
            centroids[i].second.second = sums[i][1] / counts[i];

    }
}

void kmeans(std::vector<pair<int, std::pair<double, double>>> &data, int k, int maxIterations){
    random_device rd;
    mt19937 gen(rd());
    shuffle(data.begin(), data.end(), gen);
    vector<pair<int, std::pair<double, double>>> centroids(data.begin(), data.begin() + k);

    vector<pair<int, int>> labels(data.size());

    for (int iter = 0; iter < maxIterations; ++iter) {
        assignClusters(data, centroids, labels);
        updateCentroids(data, centroids, labels);
    }

    // Imprimir os centroides e os rótulos dos clusters
    cout << "Centroides:\n";
    for (size_t i = 0; i < centroids.size(); ++i) {
        cout << "Cluster " << i + 1 << ": ";

            cout << centroids[i].second.first << " ";

        cout << "\n";
    }

    cout << "\nRótulos de Cluster:\n";
    for (size_t i = 0; i < labels.size(); ++i) {
        cout << "Ponto " << labels[i].first << " -> Cluster " << labels[i].second + 1 << "\n";
    }

    vector<vector<int>> tours = solveTSPForEachCluster(data, centroids, labels);


}


// Função para calcular a distância euclidiana entre dois pontos
double distance(const pair<int, std::pair<double, double>>& p1, const pair<int, std::pair<double, double>>& p2) {
    return sqrt((p2.second.first - p1.second.first) * (p2.second.first - p1.second.first) + (p2.second.second - p1.second.second) * (p2.second.second - p1.second.second));
}



std::vector<int> solveTSP(const std::vector<pair<int, std::pair<double, double>>>& points) {
    int n = points.size();
    std::vector<bool> visited(n, false);
    std::vector<int> tour;

    // Começar a partir do primeiro ponto
    int current = 0;
    visited[current] = true;
    tour.push_back(points[current].first);

    // Iterar até visitar todos os pontos
    while (tour.size() < n) {
        double minDistance = std::numeric_limits<double>::max();
        int next = -1;

        // Encontrar o ponto mais próximo do ponto atual
        for (int i = 0; i < n; ++i) {
            if (!visited[i]) {
                double dist = distance(points[current], points[i]);
                if (dist < minDistance) {
                    minDistance = dist;
                    next = i;
                }
            }
        }

        // Adicionar o ponto mais próximo ao tour
        visited[next] = true;
        tour.push_back(points[next].first);
        current = next;
    }

    // Adicionar o primeiro ponto novamente para formar um ciclo
    tour.push_back(tour[0]);

    return tour;
}


vector<vector<int>> solveTSPForEachCluster(std::vector<pair<int, std::pair<double, double>>>& data,
                            std::vector<pair<int, std::pair<double, double>>>& centroids,
                            std::vector<pair<int, int>>& labels) {

    // Iterar sobre cada cluster
    std::vector<std::vector<int>> tours;


    for (int i = 0; i < centroids.size(); i++) {

        std::vector<pair<int, std::pair<double, double>>> clusterPoints;


        // Extrair os pontos pertencentes ao cluster atual

        for (auto node: labels) {

            if (node.second == i) {

                for (auto v: data) {
                    if (v.first == node.first) {
                        clusterPoints.push_back(v);
                        break;
                    }
                }
            }
        }
        cout << "pontos no cluster" << endl;
        for (auto cluster: clusterPoints) {
            cout << cluster.first << endl;
        }
        cout << "proximo" << endl;
        std::vector<int> tour = solveTSP(clusterPoints);
        tours.push_back(tour);

        int n=1;
        for(auto t: tours){
            cout<<"tour "<< n<<endl;
            n++;
            for(auto v: t){
                cout<<v<<endl;
            }

        }

    }  //BacktrakingTSP(g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited){
    return tours;
}

/*void connectClusters{

};*/

//1 - começa no cluster que tem node 0
//2- organizar clusters por centroides ordem crescente
//3- ir percorrendo os clusters


/*
double distanceCentroids (pair<int, std::pair<double, double>> node1, pair<int, std::pair<double, double>> node2){
    return sqrt(pow(node1.second.first - node2.second.first, 2) + pow(node1.second.second - node2.second.second, 2));
}

void connectClusters(vector<Node>& centroids, vector<vector<int>>& adjacencyMatrix) {
    int numClusters = centroids.size();
    for (int i = 0; i < numClusters - 1; ++i) {
        double minDistance = numeric_limits<double>::max();
        int closestCluster = -1;
        for (int j = i + 1; j < numClusters; ++j) {
            double d = distance(centroids[i], centroids[j]);
            if (d < minDistance) {
                minDistance = d;
                closestCluster = j;
            }
        }
        // Adiciona a aresta entre os centroides dos clusters adjacentes ao tour completo
        adjacencyMatrix[i][closestCluster] = 1;
        adjacencyMatrix[closestCluster][i] = 1;
    }
}


// Suponha que centroids é um vetor de pontos que representam os centroides dos clusters
// e adjacencyMatrix é uma matriz de adjacência que representa o grafo completo do tour

// Função para conectar os clusters
void connectClusters(vector<Point>& centroids, vector<vector<int>>& adjacencyMatrix) {
    int numClusters = centroids.size();
    for (int i = 0; i < numClusters - 1; ++i) {
        for (int j = i + 1; j < numClusters; ++j) {
            // Encontrar os dois nós mais próximos, um de cada cluster
            pair<int, int> closestNodes = findClosestNodes(centroids[i], centroids[j]);
            int nodeFromCluster1 = closestNodes.first;
            int nodeFromCluster2 = closestNodes.second;
            // Adicionar aresta entre os nós mais próximos ao tour completo
            adjacencyMatrix[nodeFromCluster1][nodeFromCluster2] = 1;
            adjacencyMatrix[nodeFromCluster2][nodeFromCluster1] = 1;
        }
    }
}


#include <cmath>
#include <vector>
#include <utility>

struct Point {
    double x;
    double y;
};

// Função para calcular a distância euclidiana entre dois pontos
double distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// Função para encontrar os dois nós mais próximos, um de cada cluster
std::pair<int, int> findClosestNodes(const std::vector<Point>& cluster1, const std::vector<Point>& cluster2) {
    double minDistance = std::numeric_limits<double>::max();
    std::pair<int, int> closestNodes;

    for (size_t i = 0; i < cluster1.size(); ++i) {
        for (size_t j = 0; j < cluster2.size(); ++j) {
            double d = distance(cluster1[i], cluster2[j]);
            if (d < minDistance) {
                minDistance = d;
                closestNodes = std::make_pair(i, j);
            }
        }
    }

    return closestNodes;
}
*/



/*void printPrim(Graph<int>& g){
    auto vec = prim(g);
    for (auto x: vec){
        if(x->getPath()!= nullptr){
            auto e1 = x->getPath();
            for(auto y: vec){
                for (auto e2: y->getAdj()){
                    if (e2 == e1){
                        cout << "edge: " << y->getInfo() << " - " << e1->getDest()->getInfo() << endl;
                    }
                }
            }

        }
    }
}
*/



void preOrder(vector<Vertex<int> *> vertexSet, Vertex<int>* v0, vector<Vertex<int>*>& tour){
    v0->setVisited(true);
    tour.push_back(v0);
    for(auto e: v0->getAdj()){
        auto v2 = e->getDest();
        if (!v2->isVisited() and v2->getPath() == e){
            preOrder(vertexSet, v2, tour);
            tour.push_back(v2);
        }
    }
    return;
}




vector<Vertex<int> *> prim(Graph<int> * g) {
// Check if the graph is empty
    if (g->getVertexSet().empty()) {
        return g->getVertexSet(); // Return an empty set if the graph is empty
    }
// Initialize the vertices in the graph
    for(auto v : g->getVertexSet()) {
        v->setDist(INF); // Set distance to infinity
        v->setPath(nullptr); // Set path to null
        v->setVisited(false); // Mark as not visited
    }
// Select the first vertex as the starting point
    Vertex<int>* s = g->getVertexSet().front();
    s->setDist(0); // Set distance of the starting vertex to 0
// Priority queue to store vertices based on their distances
    MutablePriorityQueue<Vertex<int>> q;
    q.insert(s);
// Main loop for the Prim's algorithm
    while( ! q.empty() ) {
// Extract the vertex with the minimum distance from the priority queue
        auto v = q.extractMin();
        v->setVisited(true); // Mark the vertex as visited
// Iterate through the adjacent edges of the current vertex
        for(auto &e : v->getAdj()) {
            Vertex<int>* w = e->getDest(); // Get the destination vertex of the edge
// Check if the destination vertex is not visited
            if (!w->isVisited()) {
                auto oldDist = w->getDist(); // Get the current distance of the destination vertex
// Check if the weight of the edge is less than the current distance of the destination vertex
                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight()); // Update the distance of the destination vertex
                    w->setPath(e); // Update the path to the current edge
// If the destination vertex had infinite distance, insert it into the priority queue
                    if (oldDist == INF) {
                        q.insert(w);
                    }
// If the destination vertex had finite distance, decrease its key in the priority queue
                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }
// Return the set of vertices after the Prim's algorithm completes
    return g->getVertexSet();
}

double calculateTourDistance(const vector<Vertex<int>*>& tour) {
    double totalDistance = 0.0;
    Vertex<int>* last;

    for (int x = 0; x < tour.size() - 1; ++x) {
            for (auto &e: tour[x]->getAdj()) {
                if (e->getDest() == tour[x + 1]) {
                    last = tour[x+1];
                    totalDistance += e->getWeight();
                    break;
                }
            }
    }
    double tozero;
    for(auto x: last->getAdj()){
        if(x->getDest()->getInfo() == 0){
            tozero = x->getWeight();
        }
    }
    totalDistance += tozero;
    return totalDistance;
}

vector<Vertex<int>*> removeDup(vector<Vertex<int>*> tour){
    unordered_set<Vertex<int>*> seen;
    vector<Vertex<int>*> result;
    for (const auto& elem : tour) {
        if (seen.find(elem) == seen.end()) {
            seen.insert(elem);
            result.push_back(elem);
        }
    }
    return result;
}



void triangularApproximation(Graph<int> &g) {
    auto mst = prim(&g);

    vector<Vertex<int>*> tour;
    for(auto x: mst){
        x->setVisited(false);
    }
    preOrder(mst, mst[0], tour);

    //tour.push_back(g.findVertex(0));

    tour = removeDup(tour);cout << endl;
    double tourDistance = calculateTourDistance(tour);
    cout << "2 -Approximated distance: " << tourDistance << endl;

}

double convert_to_radians(double coord){
    return coord *M_PI/180;
}

double harversineDistance(Vertex<int>* v1, Vertex<int>* v2){

    double lon1, lat1, lon2, lat2;
    double earthradius = 6371000; // meters

    lon1 = v1->getLong();
    lat1 = v1->getLat();
    lon2=v2->getLong();
    lat2= v2->getLat();

    double rad_lat1 = convert_to_radians(lat1);
    double rad_lon1 = convert_to_radians(lon1);
    double rad_lat2 = convert_to_radians(lat2);
    double rad_lon2 = convert_to_radians(lon2);

    double delta_lat = rad_lat2 - rad_lat1;
    double delta_lon = rad_lon2 - rad_lon1;

    double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
               cos(rad_lat1) * cos(rad_lat2) *
               sin(delta_lon / 2) * sin(delta_lon / 2);
    double c = 2.0 * atan2(sqrt(a), std::sqrt(1.0 - a));

    double distance = earthradius * c;

    return distance;
}
void RealWorldFullyConnected(Graph <int>& g){
    bool found;
    for(auto &v: g.getVertexSet()){
        found = false;
        for(auto &v2: g.getVertexSet()){
            for(auto e: v->getAdj()){
                if(e->getDest()== v2)
                    found == true;
            }
            if(!found){
                double dist = harversineDistance(v, v2);
                g.addEdge(v->getInfo(), v2->getInfo(), dist);
                g.addEdge(v2->getInfo(), v->getInfo(), dist);
            }
        }
    }

}




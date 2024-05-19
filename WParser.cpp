#include "WParser.h"


vector<int> ToyGraphParser(const string& filename, Graph &g) {
    int nEdges = 0, nVertx = 0;
    vector<int> nms;
    ifstream csv("../dataset/Toy-Graphs/Toy-Graphs/" + filename +".csv");

    if (!csv.is_open()) {
        cerr << "Error opening csv file!" << endl;
    }
    unordered_set<int> seenVertices;
    string line;
    getline(csv, line);

    while (getline(csv, line)) {
        istringstream ss(line);
        string origin_str, destination_str, distance_str;

        getline(ss, origin_str, ',');
        getline(ss, destination_str, ',');
        getline(ss, distance_str, ',');

        int origin = stoi(origin_str);
        int destination = stoi(destination_str);
        double distance = stod(distance_str);

        if (seenVertices.insert(origin).second) {
            g.addVertex(origin);
            nVertx++;
        }
        if (seenVertices.insert(destination).second) {
            g.addVertex(destination);
            nVertx++;
        }

        g.addEdge(origin, destination, distance);
        g.addEdge(destination, origin, distance);
        nEdges += 2;
    }

    csv.close();
    nms.push_back(nEdges);
    nms.push_back(nVertx);
    return nms;
}

void ExtraMSGraphParser(const string& edge_filename, Graph &g){

    string line;

    ifstream nodes("../dataset/Extra_Medium_Sized_Graphs/Extra_Medium_Sized_Graphs/nodes.csv");

    if (!nodes.is_open()) {
        cerr << "Error opening nodes file!" << endl;
    }

    getline(nodes, line);

    while (getline(nodes, line)) {
        istringstream ss(line);
        string id_str, longi_str, lat_str;

        getline(ss, id_str, ',');
        getline(ss, longi_str, ',');
        getline(ss, lat_str, ',');

        int id = stoi(id_str);
        double lon = stod(longi_str);
        double lat = stod(lat_str);

        g.addVertex(id);
        auto it = g.getVertexSet().find(id);
        auto v = it->second;
        v->setLong(lon);
        v->setLat(lat);
        //(*(g.getVertexSet().end()-1))->setLong(lon);
        //(*(g.getVertexSet().end()-1))->setLat(lat);
    }
    nodes.close();

    ifstream edges("../dataset/Extra_Medium_Sized_Graphs/Extra_Medium_Sized_Graphs/"+ edge_filename + ".csv");

    if (!edges.is_open()) {
        cerr << "Error opening edges file!" << endl;
    }

    while (getline(edges, line)) {

        istringstream ss(line);
        string orig_str, dest_str, dist_str;

        getline(ss, orig_str, ',');
        getline(ss, dest_str, ',');
        getline(ss, dist_str, ',');

        int orig = stoi(orig_str);
        int dest = stoi(dest_str);
        double dist = stod(dist_str);

        g.addEdge(orig, dest, dist);
        g.addEdge(dest, orig, dist);

    }

    edges.close();

}


void RealWorldGraphParser(const string& dir_name, Graph &g){

    string line;
    ifstream nodes("../dataset/Real-World-Graphs/Real-World-Graphs/"+ dir_name + "/nodes.csv");

    if (!nodes.is_open()) {
        cerr << "Error opening nodes file!" << endl;
    }

    getline(nodes, line);

    while (getline(nodes, line)) {
        istringstream ss(line);
        string id_str, longi_str, lat_str;

        getline(ss, id_str, ',');
        getline(ss, longi_str, ',');
        getline(ss, lat_str, ',');

        int id = stoi(id_str);
        double lon = stod(longi_str);
        double lat = stod(lat_str);

        g.addVertex(id);
        auto it = g.getVertexSet().find(id);
        auto v = it->second;
        v->setLong(lon);
        v->setLat(lat);
        //(*(g.getVertexSet().end()-1))->setLong(lon);
        //(*(g.getVertexSet().end()-1))->setLat(lat);
    }

    nodes.close();
    ifstream edges("../dataset/Real-World-Graphs/Real-World-Graphs/"+ dir_name + "/edges.csv");

    if (!edges.is_open()) {
        cerr << "Error opening edges file!" << endl;
        return;
    }

    getline(edges, line);

    while (getline(edges, line)) {

        istringstream ss(line);
        string orig_str, dest_str, dist_str;

        getline(ss, orig_str, ',');
        getline(ss, dest_str, ',');
        getline(ss, dist_str, ',');

        int orig = stoi(orig_str);
        int dest = stoi(dest_str);
        double dist = stod(dist_str);

        g.addEdge(orig, dist, dest);
        g.addEdge(dist, orig, dest);

    }

    edges.close();
}


void WParser(const string& path, Graph &g){
    if (path[0] == 's' || path[0]=='t'){
        ToyGraphParser(path, g);}
    else if (path[0] == 'e')
        ExtraMSGraphParser(path, g);
    else{
        RealWorldGraphParser(path, g);
    }
}

//
// Created by saracortez on 25/04/24.
//

#include "DataParser.h"

void ToyGraphParser(const string& filename, Graph<int> &g) {
    ifstream csv("../dataset/Toy-Graphs/Toy-Graphs/" + filename + ".csv");

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
            }
            if (seenVertices.insert(destination).second) {
                g.addVertex(destination);
            }

            g.addEdge(origin, destination, distance);
            g.addEdge(destination, origin, distance);
        }

        csv.close();
}

void ExtraMSGraphParser(const string& edge_filename, Graph<int> &g){

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
        (*(g.getVertexSet().end()-1))->setLong(lon);
        (*(g.getVertexSet().end()-1))->setLat(lat);
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

        g.addEdge(orig, dist, dest);

    }

    edges.close();

}

void RealWorldGraphParser(const string& dir_name, Graph<int> &g){

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
        (*(g.getVertexSet().end()-1))->setLong(lon);
        (*(g.getVertexSet().end()-1))->setLat(lat);
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

    }

    edges.close();
}

void Parser(const string& path, Graph<int> &g, bool haversine){
    if (path[0] == 's' || path[0]=='t'){
        ToyGraphParser(path, g);}

    else{

            if (path[0] == 'e')
                ExtraMSGraphParser(path, g);

            else
                RealWorldGraphParser(path, g);
        }

    }



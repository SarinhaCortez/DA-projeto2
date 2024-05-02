//
// Created by saracortez on 25/04/24.
//

#include "DataParserMatrix.h"

void MToyGraphParser(const string& filename, MGraph &g) {
    ifstream csv("../dataset/Toy-Graphs/Toy-Graphs/" + filename);
    int maximum = 0;
    if (!csv.is_open()) {
        cerr << "Error opening csv file!" << endl;
    }
    string line;
    getline(csv, line);
    while (getline(csv, line)) {
        istringstream ss(line);
        string origin_str, destination_str, distance_str;

        getline(ss, origin_str, ',');
        getline(ss, destination_str, ',');

        int origin = stoi(origin_str);
        int destination = stoi(destination_str);

        maximum = max(maximum, max(origin, destination));
    }

    g.updateMatrixDim(maximum);
    csv.close();
    ifstream csv2("../dataset/Toy-Graphs/Toy-Graphs/" + filename);

    getline(csv2, line);
    while (getline(csv2, line)) {
        istringstream ss(line);
        string origin_str, destination_str, distance_str;

        getline(ss, origin_str, ',');
        getline(ss, destination_str, ',');
        getline(ss, distance_str, ',');

        int origin = stoi(origin_str);
        int destination = stoi(destination_str);
        double distance = stod(distance_str);

        (*g.getDistMatrix())[origin][destination] = distance;
        (*g.getDistMatrix())[destination][origin] = distance;
    }

    csv2.close();
}


void MExtraMSGraphParser(const string& edge_filename, MGraph &g){

    string line; int maximum = 0;
    ifstream edgescut("../dataset/Extra_Medium_Sized_Graphs/Extra_Medium_Sized_Graphs/"+ edge_filename+".csv");

    if (!edgescut.is_open()) {
        cerr << "Error opening edges file!" << endl;
    }

    while (getline(edgescut, line)) {

        istringstream ss(line);
        string orig_str, dest_str, dist_str;

        getline(ss, orig_str, ',');
        getline(ss, dest_str, ',');
        getline(ss, dist_str, ',');

        int orig = stoi(orig_str);
        int dest = stoi(dest_str);
        double dist = stod(dist_str);

        maximum = max(maximum, max(orig, dest));
    }
    g.updateMatrixDim(maximum);// este while loop serviu apenas para cortar o grafo
    edgescut.close();


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
        if(id > maximum) continue;
        double lon = stod(longi_str);
        double lat = stod(lat_str);

        g.addVertex(id, lon, lat);
    }
    nodes.close();

    ifstream edges("../dataset/Extra_Medium_Sized_Graphs/Extra_Medium_Sized_Graphs/"+ edge_filename+".csv");

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

        (*g.getDistMatrix())[orig][dest] = dist;
        (*g.getDistMatrix())[dest][orig] = dist;
        //edges se não existirem ficam a -1
    }

    edges.close();

}

void MRealWorldGraphParser(const string& dir_name, MGraph &g){

    string line;
    ifstream nodes("../dataset/Real-World-Graphs/Real-World-Graphs/"+ dir_name + "/nodes.csv");

    int maximum = 0;
    getline(nodes, line);

    while (getline(nodes, line)) {

        istringstream ss(line);
        string id_str, longi_str, lat_str;

        getline(ss, id_str, ',');
        getline(ss, longi_str, ',');
        getline(ss, lat_str, ',');

        int id = stoi(id_str);
        maximum = max(maximum,id);
        double lon = stod(longi_str);
        double lat = stod(lat_str);

        g.addVertex(id, lon, lat);
    }
    nodes.close();
    g.updateMatrixDim(maximum);

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

        (*g.getDistMatrix())[orig][dest] = dist;
        (*g.getDistMatrix())[dest][orig] = dist;
        //edges se não existirem ficam a -1
    }
    edges.close();
}

void Parser(const string& path, MGraph &g){
    int s = path.size();
    if(path[0] == 'e')
        MExtraMSGraphParser(path, g);
    else if(path[s-1] == 'v')
        MToyGraphParser(path, g);
    else
        MRealWorldGraphParser(path, g);
}


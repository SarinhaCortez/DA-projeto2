//
// Created by saracortez on 25/04/24.
//

#include "DataParserMatrix.h"
#include <cmath>

void MToyGraphParser(const string& filename, MGraph &g) {
    ifstream csv("../dataset/Toy-Graphs/Toy-Graphs/" + filename + ".csv" );
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
    ifstream csv2("../dataset/Toy-Graphs/Toy-Graphs/" + filename +".csv");

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
const double pi = 3.14159265358979323846;
const double earthRadius = 6371000;
double convert_radians(double coord){
return coord*pi/180.;}

int harversineDistance(int vertex1, int vertex2, MGraph &g){
    double lon1, lat1, lon2, lat2;
    auto it1 = g.getVertexSet().find(vertex1);
    if (it1 != g.getVertexSet().end()) {
        std::pair<double, double> node1 = it1->second;
        lon1 = node1.first;
        lat1 = node1.second;
    }
    auto it2 = g.getVertexSet().find(vertex2);
    if (it2 != g.getVertexSet().end()) {
        std::pair<double, double> node2 = it2->second;
        lon2 = node2.first;
        lat2 = node2.second;
    }

    double rad_lat1 = convert_radians(lat1);
    double rad_lon1 = convert_radians(lon1);
    double rad_lat2 = convert_radians(lat2);
    double rad_lon2 = convert_radians(lon2);


    double delta_lat = rad_lat2 - rad_lat1;
    double delta_lon = rad_lon2 - rad_lon1;

    double a = std::sin(delta_lat / 2) * std::sin(delta_lat / 2) +
               std::cos(rad_lat1) * std::cos(rad_lat2) *
               std::sin(delta_lon / 2) * std::sin(delta_lon / 2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    double distance = earthRadius * c;

    return distance;
}

void MExtraMSGraphParser(const string& edge_filename, MGraph &g){

    string line; int maximum = 0;

    /* o primeiro while loop serve apenas para limitar os nós a usar, caso contrário
     * a matriz fica com muito espaço inutilizado*/
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
    g.updateMatrixDim(maximum);
    edgescut.close();

    /*o segundo atualiza a latitude e a longitude dos nós usados*/
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

    /*o terceiro preenche a matriz*/
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



void MRealWorldGraphParserHaversine(const string& dir_name, MGraph &g){

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
    for(int node1=0; node1<(*g.getDistMatrix()).size(); node1++){
        for(int node2=0; node2<(*g.getDistMatrix()).size(); node2++){
            if(node1!=node2 &&  (*g.getDistMatrix())[node1][node2] == -1){
                (*g.getDistMatrix())[node1][node2] = harversineDistance(node1, node2, g);
            }
        }
    }
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

void Parser(const string& path, MGraph &g, bool haversine){
    if (path[0] == 's' || path[0]=='t'){
        MToyGraphParser(path, g);}
    else if (path[0] == 'e')
        MExtraMSGraphParser(path, g);
    else{
         if(haversine) {
                 MRealWorldGraphParserHaversine(path, g);
         }
         else{
                 MRealWorldGraphParser(path, g);
         }
    }
}


#include <iomanip>
#include <climits>
#include "menu.h"
#include "TSPSolver.h"


void Menu::openMenu() {
    cout << setw(25) << "Welcome!" << endl;
    chooseGraph();
}

Menu::Menu() {}


void Menu::wait(string graph) {
    cout << endl << "Press ENTER to continue ..." << endl;
    cin.ignore(); // Clear the input buffer

    string enter;
    getline(cin, enter); // Read the entire line including Enter key

    while (!enter.empty()) { // Keep reading lines until Enter key is pressed
        getline(cin, enter); // Get the next line of input
    }

    continueM(graph);
}
int Menu::closeMenu() {
    return 0;
}

void Menu::chooseGraph(){
    cout << "Choose which graph you want to analyze: " << endl;

    string option;
    cout << "Graph: "; cin >> option; cout << endl;

    if(option=="0"){
        initialOptions(option);
    }
    else {
        /*Parser(option, g1, true);
        Parser(option, g2, true);*/


        initialOptions(option);
    }
}

void Menu::initialOptions(string graph) {
    cout << "What do you want to consult?" << endl;
    cout << "1. T2.1\n" << "2. T2.2\n" << "3. T2.3\n"<<"4. T2.4\n"<<"5. Quit\n";
    cout << "Option: ";
    string option;
    cin >> option;

    while (!(option == "1" || option == "2" || option == "3" || option == "4" || option=="5")) {
        cout << "Invalid input. Option: ";
        cin >> option;
    }

    cout << " " << endl;
    if (option == "1") {
        if(g1.getNumVertex()==0){
        Parser(graph, g1, true);}
        int initial =0;
        std::vector<int> tsp;
        std::vector<int> final;
        int numVertex=g1.getNumVertex();
        tsp.push_back(initial);
        int dist=0;
        int minDist  = INT_MAX;
        bool found=false;

        std::set<int>visited;

        if(BacktrakingTSP(g1, dist, minDist, tsp, final, numVertex, initial, found, visited)){
            double totalMinDist=0;
            std::vector<std::vector<double>>* matrix = g1.getDistMatrix();
            for(int i=0; i<final.size()-1; i++){
                totalMinDist+=(*matrix)[final[i]][final[i+1]];
                cout<<final[i]<<endl;
            }
            cout<<"0"<<endl;
            cout<<"Total MIn Dist: "<<totalMinDist<<endl;
        }
        else{
            cout<<"nao existe lol"<<endl;
        }

        cout<<endl;
        wait(graph);
    }
    if (option == "2") {
        Parser(graph, g3, true);
        triangularApproximation(g3);
        cout<<endl;
        wait(graph);
    }
    if (option == "3") {
        if(g1.getNumVertex()==0){
            Parser(graph, g1, false);}
        std::vector<std::vector<double>>* matrix = g1.getDistMatrix();
        std::vector<int> tsp;
        tsp.push_back(0);
        nearestNeighbour(g1, tsp, matrix);

        cout<<endl;
        wait(graph);
    }

    if (option == "4") {
        if(g2.getNumVertex()==0){
            Parser(graph, g2, false);}
        cout << "Loading your answer as fast as a blast..." <<endl;
        TSPSolver T(g2, 10000, 0.99, 10000);
        cout << "Here's the Traveling Sales-Woman Journey!";
        double bestCost;
        std::vector<int> res = T.solve(bestCost);
        for(auto el:res){
            cout  << el<<", ";
        }
        cout << endl;
        cout << "And here's how long she traveled: " << bestCost;
        wait(graph);
    }

    if (option == "5") { closeMenu(); }
}

void Menu::continueM(string graph){
    cout << "Choose one option" << endl;
    cout << "1. Go back to Menu\n" << "2.Quit\n" << endl;
    cout << "Option: ";
    string option;
    cin >> option;

    while (!(option == "1" || option == "2") ){
        cout << "Invalid input. Option: ";
        cin >> option;
    }
    if( option == "1"){initialOptions(graph);}
    else{closeMenu();}
}
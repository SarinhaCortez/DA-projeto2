#include <iomanip>
#include "menu.h"

void Menu::openMenu() {
    cout << setw(25) << "Welcome!" << endl;
    chooseGraph();
}

Menu::Menu() {}


void Menu::wait() {
    cout << endl << "Press ENTER to continue ..." << endl;
    cin.ignore(); // Clear the input buffer

    string enter;
    getline(cin, enter); // Read the entire line including Enter key

    while (!enter.empty()) { // Keep reading lines until Enter key is pressed
        getline(cin, enter); // Get the next line of input
    }

    continueM();
}
int Menu::closeMenu() {
    return 0;
}

void Menu::chooseGraph(){
    cout << "Choose which graph you want to analyze: " << endl;

    string option;
    cout << "Graph: "; cin >> option; cout << endl;

    Parser(option, g1, true);
    Parser(option, g2, true);
    initialOptions();
}

void Menu::initialOptions() {
    cout << "What do you want to consult?" << endl;
    cout << "1. T2.1\n" << "2. T2.2\n" << "3. T2.3\n"<<"4. 2.4\n"<<"5. Quit\n";
    cout << "Option: ";
    string option;
    cin >> option;

    while (!(option == "1" || option == "2" || option == "3" || option == "4" || option=="5")) {
        cout << "Invalid input. Option: ";
        cin >> option;
    }

    cout << " " << endl;
    if (option == "1") {
        int initial =0;
        std::vector<int> tsp;
        std::vector<int> final;
        int numVertex=g1.getNumVertex();
        tsp.push_back(initial);
        int dist=0;
        int minDist=INT_MAX;
        bool found=false;

        std::set<int>visited;

        if(BacktrakingTSP(g1, dist, minDist, tsp, final, numVertex, initial, found, visited)){
            int totalMinDist=0;
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
        wait();
    }
    if (option == "2") {
        for(auto v : (*g1.getDistMatrix())){
      for(auto d : v){
          cout << d <<" ";
      }cout << endl;
  }
        wait();
    }
    if (option == "3") {
        std::vector<std::vector<double>>* matrix = g1.getDistMatrix();
        std::vector<int> tsp;
        tsp.push_back(0);
        nearestNeighbour(g1, tsp, matrix);

        cout<<endl;
        wait();
    }

    if (option == "4") {
        cout<<"A ser tratado"<<endl;
        wait();
    }

    if (option == "5") { closeMenu(); }
}

void Menu::continueM(){
    cout << "Choose one option" << endl;
    cout << "1. Go back to Menu\n" << "2.Quit\n" << endl;
    cout << "Option: ";
    string option;
    cin >> option;

    while (!(option == "1" || option == "2") ){
        cout << "Invalid input. Option: ";
        cin >> option;
    }
    if( option == "1"){initialOptions();}
    else{closeMenu();}
}
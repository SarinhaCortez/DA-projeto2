#ifndef DA_PROJETO2_MENU_H
#define DA_PROJETO2_MENU_H

#include "WParser.h"
#include "WGraph.h"
#include "DataParserMatrix.h"
#include "Utils.h"
#include "TSPSolver.h"
#include <climits>
#include <iomanip>




class Menu{
public:
    MGraph g1; //com haversine distance automática
    MGraph g2; //sem haversine distance automática
    Graph g3; //aulas

    Menu();
    void buildGraph(string graph);
    void openMenu();
    void wait(string graph);
    void initialOptions(string graph);
    int closeMenu();
    void chooseGraph();
    void continueM(string graph);
};


#endif //DA_PROJETO2_MENU_H

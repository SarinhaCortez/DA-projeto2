#ifndef DA_PROJETO2_MENU_H
#define DA_PROJETO2_MENU_H

#include "DataParser.h"
#include <climits>
#include "DataParserMatrix.h"
#include "Utils.h"


class Menu{
public:
    MGraph g;
     Menu();
    void buildGraph(string graph);
    void openMenu();
    void wait();
    void initialOptions();
    int closeMenu();
    void chooseGraph();
    void continueM();
};


#endif //DA_PROJETO2_MENU_H

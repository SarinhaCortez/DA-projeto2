#ifndef DA_PROJETO2_MENU_H
#define DA_PROJETO2_MENU_H

#include "WParser.h"
#include "WGraph.h"
#include "DataParserMatrix.h"
#include "Utils.h"
#include "TSPSolver.h"
#include <climits>
#include <iomanip>



/**
 * @brief Class of the menu system for analyzing graphs.
 *
 * The Menu class provides functionality to build and interact with various
 * types of graphs. It supports operations such as selecting graphs, displaying menu
 * options, and performing specific algorithms on the graphs.
 */
class Menu{
public:
    MGraph g1; //com haversine distance automática
    MGraph g2; //sem haversine distance automática
    Graph g3;
    bool parsed3;

    /**
    * @brief Constructs a Menu object.
    */
    Menu();

    void openMenu();
    /**
     * @brief Waits for user input to continue.
     *
     * @param graph The name of the graph being analyzed.
     */
    void wait(string graph);

    /**
     * @brief Displays the initial options menu.
     *
     * Menu where the user can select the algorithm to perform.
     *
     * @param graph The name of the graph being analyzed.
     */
    void initialOptions(string graph);

    /**
    * @brief Closes the menu.
    */
    int closeMenu();

    /**
     * @brief Prompts the user to choose a graph to analyze.
     */
    void chooseGraph();

    /**
     * @brief Continues menu interaction after a wait period.
     *
     * @param graph The name of the graph being analyzed.
     */
    void continueM(string graph);

    //void buildGraph(string graph);
};


#endif //DA_PROJETO2_MENU_H

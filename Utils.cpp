    #include "Utils.h"
    #include "data_structures/Graph.h"
    #include <vector>

    using namespace std;

    bool BacktrakingTSP(Graph<int> &g, int &dist, int &minDist, std::vector<Vertex<int> *> &tsp, std::vector<Vertex<int> *> &final, int &numVertex, Vertex<int> * vertex, bool&found){

        if(vertex->getInfo()==(g.getNumVertex()-1) && numVertex==1){

            if(dist<minDist){
                minDist=dist;
                final=tsp;


                found=true;
                return found;
            }


            return false;
        }

        else if(numVertex>1 && vertex->getInfo()!=(g.getNumVertex()-1)) {

            for (auto edge: vertex->getAdj()) {

                Vertex<int> *dest = edge->getDest();

                if (!dest->isVisited()) {
                    dest->setVisited(true);
                    tsp.push_back(dest);

                    int newNumVertex=numVertex - 1;
                    int newDist=dist + edge->getWeight();

                    if (BacktrakingTSP(g, newDist, minDist, tsp, final, newNumVertex, dest, found)){
                        dest->setVisited(false);
                        tsp.pop_back();
                    }
                    else{
                        dest->setVisited(false);
                        tsp.pop_back();
                        return false;}

                }
            }
        }

        else{
            return found;
        }
        return found;
    }
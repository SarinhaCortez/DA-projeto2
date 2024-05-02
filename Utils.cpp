    #include "Utils.h"
    #include "data_structures/Graph.h"
    #include <vector>

    using namespace std;

    bool BacktrakingTSP(Graph<int> &g, int &dist, int &minDist, std::vector<Vertex<int> *> &tsp, std::vector<Vertex<int> *> &final, int &numVertex, Vertex<int> * vertex, bool&found){
        bool final_found = false;
        if(vertex->getInfo()==0 && numVertex==0 && vertex->isVisited()){

            if(dist<minDist){
                minDist=dist;
                final.clear();
                for(auto element: tsp){
                    final.push_back(element);
                }


                found=true;
                return found;
            }

            return false;
        }

        else if((numVertex>0 && vertex->getInfo()!=0) || (!vertex->isVisited() && vertex->getInfo()==0 && numVertex>0)) {
            cout << vertex->getInfo()<<" entrei"<<endl;
            for (auto edge: vertex->getAdj()) {

                Vertex<int> *dest = edge->getDest();

                cout<<dest->getInfo()<<endl;
                cout<<"vertices que faltam: "<<numVertex<<endl;
                if ((!dest->isVisited() && dest->getInfo()!=0) || (!dest->isVisited() && dest->getInfo()==0 && numVertex==1)) {
                    dest->setVisited(true);
                    tsp.push_back(dest);

                    int newNumVertex=numVertex - 1;
                    int newDist=dist + edge->getWeight();
                    cout<<dest->getInfo()<<endl;
                    if (BacktrakingTSP(g, newDist, minDist, tsp, final, newNumVertex, dest, found)){
                        cout<<vertex->getInfo()<<" ----> "<<dest->getInfo()<<" ola"<<endl;
                        dest->setVisited(false);
                        tsp.pop_back();
                        final_found=true;
                        /*dist = newDist-edge->getWeight();
                        numVertex++;*/
                    }
                    else{
                        cout<<vertex->getInfo()<<" ----> "<<dest->getInfo()<<" adeus"<<endl;
                        dest->setVisited(false);
                        tsp.pop_back();
                        /*dist = newDist-edge->getWeight();
                        numVertex++;*/
                        return found;}
                }
            }
        }
        else{
            cout<<vertex->getInfo() << "----> retorna falso"<<endl;
            return false;
        }
        vertex->setVisited(false);
        cout<<"aqui em baixo haha : ----->"<< final_found<<endl;
        return final_found;
    }
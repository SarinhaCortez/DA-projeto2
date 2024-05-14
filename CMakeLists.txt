cmake_minimum_required(VERSION 3.26)
project(DA_projeto2)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(data_structures)

add_executable(DA_projeto2
        data_structures/Graph.h
        data_structures/MutablePriorityQueue.h
        data_structures/UFDS.cpp
        data_structures/UFDS.h
        DataParser.cpp
        DataParser.h
        main.cpp
        Utils.cpp
        data_structures/MatrixGraph.h
        DataParserMatrix.cpp
        DataParserMatrix.h
        menu.h
        menu.cpp
        haversine.cpp
        haversine.h
        TSPSolver.h)

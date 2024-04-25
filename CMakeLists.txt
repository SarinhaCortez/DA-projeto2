cmake_minimum_required(VERSION 3.28)
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
        main.cpp)

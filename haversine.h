//
// Created by saracortez on 10/05/24.
//

#ifndef DA_PROJETO2_HAVERSINE_H
#define DA_PROJETO2_HAVERSINE_H
#include <math.h>

constexpr double PI = 3.14159265358979323846;
using angle_t      = double;
using radians_t    = double;
using kilometers_t = double;

auto calculate_distance(const angle_t latitude1,
                        const angle_t longitude1,
                        const angle_t latitude2,
                        const angle_t longitude2) -> kilometers_t;

auto convert(const angle_t angle) -> radians_t;

#endif //DA_PROJETO2_HAVERSINE_H

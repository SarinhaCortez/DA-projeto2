//
// Created by saracortez on 10/05/24.
//

#ifndef DA_PROJETO2_HAVERSINE_H
#define DA_PROJETO2_HAVERSINE_H
#include <math.h>


/**
 * @brief Class suggested by the Lecturer in the Project Descripiton, to calculate the Haversine Distance between two geographical coordinates.
 * Taken from https://github.com/AhiyaHiya/haversine
 *
 *	convert_to_radians(coord):
 *		return coord*pi/180;
 *	Haversine([lat1, lon1], [lat2, lon2]):
 *		rad_lat1, rad_lon1, rad_lat2, rad_lon2 = convert_to_radians(lat1), convert_to_radians(lon1),
 *		convert_to_radians(lat2), convert_to_radians(lon2)
 *		delta_lat = rad_lat2 - rad_lat1
 *		delta_lon = rad_lon2 - rad_lon1
 *		aux = sin^2(delta_lat/2) + cos(rad_lat1) * cos(rad_lat2) * sin^2(delta_lon/2)
 *		c = 2.0 * atan2 ( sqrt(a), sqrt(1.0-a))
 *		earthradius = 6371000 (in meters)
 *		return earthradius * c
 */
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

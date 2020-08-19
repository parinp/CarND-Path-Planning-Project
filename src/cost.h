#ifndef COST_H
#define COST_H

#include "vehicle.h"

using namespace std;

void plan(Vehicle &car, vector<vector<double>> sensor_fusion, const int prev_size);

void change_param(Vehicle &car, const vector<vector<double>> sensor_fusion, const double car_yaw);



#endif
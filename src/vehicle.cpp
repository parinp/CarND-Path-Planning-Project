#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float target_v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->target_v = target_v;
  this->a = a;
  this->state = state;
  
}

Vehicle::~Vehicle() {}
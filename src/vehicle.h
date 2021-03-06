#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;
using std::map;

class Vehicle{
public:
    //Constructor
    Vehicle();
    Vehicle(int lane, float s, float v, float target_v, float a, string state);

    // Destructor
    virtual ~Vehicle();

    //Vehicle Functions
    



    //public vehicle variables

    vector<string> possible_states;
                                       
    int lane, target_lane, s;
    float v, target_v, a, max_acceleration;
    
    string state, next_state;

};

#endif
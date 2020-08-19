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
    struct collider {
        bool collisionRight;
        bool collisionLeft;
        int time;
    };

    vector<string> possible_states;

    map<string,vector<string>> transitions = {{"KL",{"ACC","PLCL","PLCR"}},
                                              {"PLCL",{"LCL"}},
                                              {"PLCR",{"LCR"}},
                                              {"LCL",{"LCL"}},
                                              {"LCR",{"LCR"}},
                                              {"ACC",{"KL","PLCL","PLCR"}}};
                                       
    int lane, target_lane, s;
    float v, target_v, a, max_acceleration;

    
    string state;
};

#endif
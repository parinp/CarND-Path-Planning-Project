#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm> // for std::find

using std::find;
using std::end;
using std::begin;

void plan(Vehicle &car, vector<vector<double>> sensor_fusion, const int prev_size){
    
    double front_car = 9999999.0;
    double front_car_speed = 0;

    bool left_car = false;
    bool right_car = false;
    double safe_distance = 40.0; 
    double distance_right = 100000;
    double distance_left = 100000;

    for(int i = 0; i < sensor_fusion.size();i++)
    {
        double d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];    
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += ((double)prev_size*0.02*check_speed);
        
        double dist_to_car = check_car_s - car.s;

        if( d < 2+4*car.lane+2 && d > 2+4*car.lane-2)
        {
            double front = dist_to_car;
            if(front > 0 && front < front_car)
            {
                front_car = front;
                front_car_speed = check_speed * 2.24;//2.24for mph 

            }
            if((check_car_s > car.s) && (dist_to_car < safe_distance))
            {
                car.state = "KL";
                car.target_v = front_car_speed;    
                
            }
        }else
        if(car.lane>0 && d < 2+4*(car.lane-1)+2 && d>2+4*(car.lane-1)-2)
        {
            if(dist_to_car < safe_distance && check_car_s > car.s)
            {
                left_car = true;
            }else
            if(dist_to_car > safe_distance)
            {
                if(dist_to_car < distance_left)
                {
                    distance_left = dist_to_car;
                }
            }

            if(fabs(dist_to_car)<15)
            {
                left_car = true;
            }
        }else
        if(car.lane<2 && d < 2+4*(car.lane+1)+2 && d>2+4*(car.lane+1)-2)
        {
            if(dist_to_car < safe_distance && check_car_s > car.s)
            {
                right_car = true;
            }else
            if(dist_to_car > safe_distance)
            {
                if(dist_to_car < distance_right)
                {
                    distance_right = dist_to_car;
                }
            }

            if(fabs(dist_to_car)<15)
            {
                right_car = true;
            }
        }else
        if(car.lane == 2)
        {
            right_car = true;
        }else
        if(car.lane == 0)
        {
            left_car = true;
        }
        
    }//end of sensor fusion loop

    if(car.state == "KL" && car.next_state != "LCL" && car.next_state != "LCR")
    {
        if(left_car == false && right_car == true)
        {
            car.next_state = "LCL";
            car.target_lane -= 1 ;
        }else
        if(left_car == true && right_car == false)
        {
            car.next_state = "LCR";
            car.target_lane += 1 ;
        }else
        if(left_car == true && right_car == true)
        {
            car.next_state = "";
        }else
        if(left_car == false && right_car == false)
        {
            if(distance_left>distance_right)
            {
                car.next_state = "LCL";
                car.target_lane -= 1 ;
            }else
            if(distance_left<distance_right)
            {
                car.next_state = "LCR";
                car.target_lane += 1 ;
            }else
            if(distance_left==distance_right)
            {
                int arrayNum[2] = {-1, 1};
                int RandIndex = rand() % 2; 
                int decision = arrayNum[RandIndex];

                if(decision == -1)
                {
                    car.next_state = "LCL";
                    car.target_lane -= 1 ;    
                }else
                {
                    car.next_state = "LCR";
                    car.target_lane += 1 ;    
                }
            }
        }
    }
    
    //std::cout << "Left: " << left_car << endl;
    //std::cout << "Right: " << right_car << endl;

    if(front_car > 50)
    {
        car.state = "ACC";
        car.next_state = "";
    }
 
}

void change_param(Vehicle &car, const vector<vector<double>> sensor_fusion, const double car_d)
{

    //std::cout << car.state << " > " << car.next_state << endl;
    //std::cout << car.lane << " > " << car.next_state << endl;

    if(car.state == "KL")
    {   
        if(car.next_state == "LCL" || car.next_state == "LCR")
        {

            if(car.v < 49.0)
            {
                car.v += car.a;
            }
            if( car_d < 2+4*car.target_lane+2 && car_d > 2+4*car.target_lane-2)
            {
                car.state = "ACC";
                car.lane = car.target_lane;
                car.next_state = "";
            }
        }else
        {
            if(car.v > car.target_v)
            {
                car.v -= car.a;
            }else 
            {
                car.v = car.target_v;
            }
        }

    }else
    if(car.state == "ACC")
    {
        if(car.v < 49.0)
        {
            car.v += car.a;
        }
    }
}
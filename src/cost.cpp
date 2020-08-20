#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>

void plan(Vehicle &car, vector<vector<double>> sensor_fusion, const int prev_size){
    
    Vehicle::collider isCollide;
    isCollide.collisionLeft = false;
    isCollide.collisionRight = false;

    double safe_distance = 30.0; 
    double free_distance_left = 9999999.0; 
    double free_distance_right= 9999999.0;
    

    for(int j = 0; j < car.possible_states.size();j++)
    {
        
        string pred_state = car.possible_states[j];

        for(int i = 0; i < sensor_fusion.size();i++)
        {
            double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];    
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size*0.02*check_speed);
            
            
            if(pred_state == "ACC")
            {
                if( d < 2+4*car.lane+2 && d > 2+4*car.lane-2)
                {
                    if((check_car_s > car.s) && ((check_car_s-car.s) > 50))
                    {
                        car.state = pred_state;
                    }
                }
            }else
            if(pred_state == "KL")
            {
                
                if( d < 2+4*car.lane+2 && d > 2+4*car.lane-2)
                {
                    if((check_car_s > car.s) && ((check_car_s-car.s) < safe_distance))
                    {
                        car.state = pred_state;
                        car.target_v = check_speed*2.24;   //2.24for mph  
                    }
                }
            }else
            if(pred_state == "PLCL")
            {
                if(car.lane>0 && d < 2+4*(car.lane-1)+2 && d>2+4*(car.lane-1)-2)
                {
                    if((check_car_s-car.s)<free_distance_left && (check_car_s > car.s))
                    {
                        free_distance_left = check_car_s - car.s;
                    }

                    if(abs(check_car_s - car.s)<10)
                    {
                        isCollide.collisionLeft = true;
                    }
                }

                if(car.lane==0)
                {
                    isCollide.collisionLeft = true;
                }

            }else
            if(pred_state == "PLCR")
            {
                if(car.lane<2 && d < 2+4*(car.lane+1)+2 && d>2+4*(car.lane+1)-2)
                {
                    if((check_car_s-car.s)<free_distance_right && (check_car_s > car.s))
                    {
                        free_distance_right = check_car_s - car.s;
                    }

                    if(abs(check_car_s - car.s)<10)
                    {
                        isCollide.collisionRight = true;
                    }
                }

                if(car.lane==2)
                {
                    isCollide.collisionRight = true;
                }
            } 
        }
    }

    free_distance_left = isCollide.collisionLeft? 0:free_distance_left;
    free_distance_right = isCollide.collisionRight? 0:free_distance_right;

    
    if(free_distance_left == free_distance_right)
    {
        int choose = rand() %1;
        if(choose == 0)
        {
            free_distance_right = 0;
        }else
        {
            free_distance_left = 0;
        }
    }
    
    
    if(car.state=="KL")
    {
        //std::cout << "free left: " << free_distance_left << endl;
        //std::cout << "free right: " << free_distance_right << endl;
        
        if(free_distance_left>free_distance_right && free_distance_left>(safe_distance*1))
        {
            car.state = car.transitions["PLCL"][0];
            car.target_lane -= 1;
            //std::cout << "turn left"<< endl;
        }else
        if(free_distance_left<free_distance_right && free_distance_right>(safe_distance*1))
        {
            car.state = car.transitions["PLCR"][0];
            car.target_lane += 1;
            //std::cout << "turn right" << endl;
        }
    }   
}

void change_param(Vehicle &car, const vector<vector<double>> sensor_fusion, const double car_d)
{
    //std::cout << car.lane << endl;
    if(car.state == "KL")
    {
        
        if(car.v > car.target_v)
        {
            car.v -= car.a;
        }else 
        {
            car.v = car.target_v;
        }
    }else
    if(car.state == "ACC")
    {
        if(car.v < 49.0)
        {
            car.v += car.a;
        }
    }else
    if(car.state == "LCL" || car.state == "LCR")
    {

        if(car.v < 49.0)
        {
            car.v += car.a;
        }
        if( car_d < 2+4*car.target_lane+2 && car_d > 2+4*car.target_lane-2)
        {
            car.state = "ACC";
            car.lane = car.target_lane;
        }
    }
}
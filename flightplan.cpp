//
//  flightplan.cpp
//  waypoints_circular
//
//  Created by Harvard Virgil Humphrey on 2019/10/18.
//  Copyright © 2019 Harvard Virgil Humphrey. All rights reserved.
//
#include <iostream>
#include <stdio.h>
#include <vector>
#include "flightplan.h"
#include "math.h"

missionplan circleflightplan(double radius,int n,double target_height)    {
    //std::cout<<C_PI<<std::endl;
    missionplan flightplan;
    double m = n*1.0;
    for(int i=0;i<n;i++)    {
        double theta = (i/m)*2*C_PI;
        vector3D waypoint;
        waypoint.x = radius*cos(theta);
        waypoint.y = radius*sin(theta);
        waypoint.z = target_height;
        flightplan.waypts.push_back(waypoint);
        //std::cout<<i<<std::endl;
        //std::cout<<theta<<std::endl;
        //std::cout<<flightplan.waypts.at(i).x<<" "<<flightplan.waypts.at(i).y<<" "<<flightplan.waypts.at(i).z<<std::endl;
    }
    flightplan.nwaypoints = m;
    flightplan.mode = 1;
    return flightplan;
}

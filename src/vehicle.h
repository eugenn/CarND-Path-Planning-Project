//
// Created by Eugen Nekhai on 24/10/2017.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <fstream>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "spline.h"

using namespace std;

using std::vector;
using std::map;

int DToLane(double d);

class Vehicle {
public:
    explicit Vehicle(const vector<double> &sensor_fusion);

    ~Vehicle();

    int id;
    int lane;
    double d;
    double s;
    double speed;
};

// Type for saving state between evaluation cycles with the simulator
struct state_t {
    state_t() {
        last_s = 0.0;
        last_d = 0.0;
        last_speed = 0.0;
    };

    state_t(double s, double d, double speed) {
        last_s = s;
        last_d = d;
        last_speed = speed;
    }

    double last_s;
    double last_d;
    double last_speed;
};

struct telemetry_t {
    telemetry_t(state_t &state, vector<Vehicle> &vehicles) {
        this->lane = DToLane(state.last_d);
        this->s = state.last_s;
        this->speed = state.last_speed;
        this->vehicles = vehicles;
    }

    int lane;
    double s;
    double speed;
    vector<Vehicle> vehicles;

};


#endif //PATH_PLANNING_VEHICLE_H

#include "vehicle.h"

int DToLane(double d) {
    if (d < 4) {
        return 1;
    }
    if (d >= 4.0 && d < 8.0) {
        return 2;
    }
    if (d >= 8.0) {
        return 3;
    }
    return 0;
}

Vehicle::~Vehicle() {

}

Vehicle::Vehicle(const vector<double> &sensor_fusion) {
    this->id = sensor_fusion[0];
    this->s = sensor_fusion[5];
    this->d = sensor_fusion[6];
    this->lane = DToLane(d);
    double vx = sensor_fusion[3];
    double vy = sensor_fusion[4];
    this->speed = sqrt(vx * vx + vy * vy);
}


#define LANE_CHANGE_COST_SIDE_F 0.9
#define LANE_CHANGE_COST_SIDE_R 0.5
#define LANE_CHANGE_COST_AHEAD  0.9

#define MAX_COST                1000.0

#define PATH_PLAN_SECONDS       2.0
#define PATH_PLAN_INCREMENT     0.02


#define MAX_SPEED_M_S           19.5
#define MIN_SPEED_M_S           10.0

#define DISTANCE_ADJUSTMENT     4.0
#define DISTANCE_THRESHOLD      25.0


#define MIN_TRACKING_CHANGE    -4.0
#define MAX_TRACKING_CHANGE     4.0

#define MAX_TRACK_S             6945.554

#include <fstream>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using json = nlohmann::json;

double LaneToD(int lane);

struct path_t {
    vector<double> path_x;
    vector<double> path_y;
    double last_s;
    double last_d;
};

enum ACTION {
    LEFT, KEEP, RIGHT
};

// Setpoint type for the controls we are returning
struct projection_t {
    double start_s;
    double start_speed;
    double end_s;
    double end_speed;
    int src_lane;
    int trg_lane;
};

class PathPlanner {

public:
    PathPlanner();

    ~PathPlanner();

    void Init(state_t state);

    double ChangeLeftCost(telemetry_t &telemetry_data);

    double KeepLaneCost(telemetry_t &telemetry_data);

    double ChangeRightCost(telemetry_t &telemetry_data);

    ACTION NextAction(telemetry_t &telemetry_data);

    path_t Path(vector<Vehicle> &vehicles, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                vector<double> &map_waypoints_y);

private:
    state_t state;

    double ClosestDistanceInFront(vector<Vehicle> &vehicles, double s, int lane);

    double ClosestDistanceBehind(vector<Vehicle> &vehicles, double s, int lane);

    projection_t LeftCourse(telemetry_t &telemetry_data);

    projection_t RightCourse(telemetry_t &telemetry_data);

    projection_t StraightCourse(telemetry_t &telemetry_data);

    vector<double> computeMinimumJerk(vector<double> start, vector<double> end, double max_time, double time_inc);

    vector<double>
    getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    path_t JerkPath(projection_t &new_setpoints, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                    vector<double> &map_waypoints_y);

};

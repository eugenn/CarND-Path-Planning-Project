//
// Created by Eugen Nekhai on 24/10/2017.
//

#include "planner.h"

PathPlanner::PathPlanner() {

}

PathPlanner::~PathPlanner() {

}

// For converting back and forth between radians and degrees.
//constexpr double pi() { return M_PI; }


void PathPlanner::Init(state_t state) {
    this->state = state;
}

// Determine distance to closest car in front of us in a given lane
double PathPlanner::ClosestDistanceInFront(vector<Vehicle> &vehicles, double s, int lane) {
    double closest = 1000000.0;
    for (auto &vehicle : vehicles) {
        if (vehicle.lane == lane) {

            double diff = vehicle.s - s;

            if (diff > 0.0 && diff < closest) {
                closest = diff;
            }
        }
    }
    return closest;
}

// Determine distance to the closest car behind us in a given lane
double PathPlanner::ClosestDistanceBehind(vector<Vehicle> &vehicles, double s, int lane) {
    double closest = 1000000.0;
    for (auto &vehicle : vehicles) {
        if (vehicle.lane == lane) {

            double diff = s - vehicle.s;

            if (diff > 0.0 && diff < closest) {
                closest = diff;
            }
        }
    }
    return closest;
}

ACTION PathPlanner::NextAction(telemetry_t &telemetry_data) {
    double left_cost = ChangeLeftCost(telemetry_data);
    double keep_cost = KeepLaneCost(telemetry_data);
    double right_cost = ChangeRightCost(telemetry_data);

    cout << "costs: " << left_cost << " - " << keep_cost << " - " << right_cost << endl;

    map<double, ACTION> cost_map = {{left_cost,  LEFT},
                                    {keep_cost,  KEEP},
                                    {right_cost, RIGHT}};

    // First value is the lowest cost since it is a priority queue on key
    map<double, ACTION>::iterator cost_map_iterator;

    cost_map_iterator = cost_map.begin();

    return cost_map_iterator->second;
}

// Cost of a change of lane to the left
double PathPlanner::ChangeLeftCost(telemetry_t &telemetry_data) {
    if (telemetry_data.lane == 1) {
        return MAX_COST;
    }

    vector<Vehicle> &vehicles = telemetry_data.vehicles;
    double &s = telemetry_data.s;
    int lane = telemetry_data.lane - 1;

    double front_dist = ClosestDistanceInFront(vehicles, s, lane);
    double behind_dist = ClosestDistanceBehind(vehicles, s, lane);

    if (front_dist != 0.0 && behind_dist != 0.0) {
        return LANE_CHANGE_COST_SIDE_F / front_dist + LANE_CHANGE_COST_SIDE_R / behind_dist;
    }

    return MAX_COST;
}

// Cost of a change of lane to the right
double PathPlanner::ChangeRightCost(telemetry_t &telemetry_data) {
    if (telemetry_data.lane == 3) {
        return MAX_COST;
    }

    vector<Vehicle> &vehicles = telemetry_data.vehicles;
    double &s = telemetry_data.s;
    int lane = telemetry_data.lane + 1;

    double front_dist = ClosestDistanceInFront(vehicles, s, lane);
    double behind_dist = ClosestDistanceBehind(vehicles, s, lane);

    if (front_dist != 0.0 && behind_dist != 0.0) {
        return LANE_CHANGE_COST_SIDE_F / front_dist + LANE_CHANGE_COST_SIDE_R / behind_dist;
    }
    return MAX_COST;
}

// Cost of maintaining straight course
double PathPlanner::KeepLaneCost(telemetry_t &telemetry_data) {
    vector<Vehicle> &vehicles = telemetry_data.vehicles;
    double &s = telemetry_data.s;
    int lane = telemetry_data.lane;

    double front_dist = ClosestDistanceInFront(vehicles, s, lane);

    if (telemetry_data.speed < 1.0) {
        return 0.0;
    }

    if (front_dist != 0.0) {
        return LANE_CHANGE_COST_AHEAD / front_dist;
    }

    return MAX_COST;
}

// Determine new Path whilst going on the left course
projection_t PathPlanner::LeftCourseSetpoints(telemetry_t &telemetry_data) {
    projection_t retval = {
            telemetry_data.s,
            telemetry_data.speed,
            telemetry_data.s + PATH_PLAN_SECONDS * telemetry_data.speed,
            telemetry_data.speed,
            telemetry_data.lane,
            telemetry_data.lane - 1
    };
    return retval;
}

// Determine new Path whilst going on the right course
projection_t PathPlanner::RightCourseSetpoints(telemetry_t &telemetry_data) {
    projection_t retval = {
            telemetry_data.s,
            telemetry_data.speed,
            telemetry_data.s + PATH_PLAN_SECONDS * telemetry_data.speed,
            telemetry_data.speed,
            telemetry_data.lane,
            telemetry_data.lane + 1
    };
    return retval;
}

// Determine new Path whilst going on the straight course
projection_t PathPlanner::StraightSetpoints(telemetry_t &telemetry_data) {
    vector<Vehicle> &vehicles = telemetry_data.vehicles;
    double &s = telemetry_data.s;
    int lane = telemetry_data.lane;

    double car_in_front_dist = ClosestDistanceInFront(vehicles, s, lane);
    double car_in_front_adj = DISTANCE_ADJUSTMENT * (car_in_front_dist - DISTANCE_THRESHOLD);

    if (car_in_front_adj > MAX_TRACKING_CHANGE) {
        car_in_front_adj = MAX_TRACKING_CHANGE;
    }

    if (car_in_front_adj < MIN_TRACKING_CHANGE) {
        car_in_front_adj = MIN_TRACKING_CHANGE;
    }

    double speed_start = telemetry_data.speed;

    double speed_end = speed_start + car_in_front_adj;

    if (speed_end > MAX_SPEED_M_S) {
        speed_end = MAX_SPEED_M_S;
    }

    if (speed_end < MIN_SPEED_M_S) {
        speed_end = MIN_SPEED_M_S;
    }

    cout << "car_in_front_dist=" << car_in_front_dist << "car_in_front_adj=" << car_in_front_adj << endl;
    cout << "speed_start=" << speed_start << "speed_end=" << speed_end << endl;
    
    projection_t retval = {
            telemetry_data.s,
            speed_start,
            telemetry_data.s + PATH_PLAN_SECONDS * 0.5 * (speed_start + speed_end),
            speed_end,
            lane,
            lane
    };
    return retval;
}

path_t PathPlanner::Path(vector<Vehicle> &vehicles, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                         vector<double> &map_waypoints_y) {
    telemetry_t telemetry_data = {state, vehicles};
    projection_t points{};

    switch (NextAction(telemetry_data)) {
        case LEFT:
            points = LeftCourseSetpoints(telemetry_data);
            break;
        case RIGHT:
            points = RightCourseSetpoints(telemetry_data);
            break;
        case KEEP:
            points = StraightSetpoints(telemetry_data);
            break;
    }

    path_t path = JerkPath(points, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    state.last_s = path.last_s;
    state.last_d = path.last_d;
    state.last_speed = points.end_speed;

    return path;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }


double LaneToD(int lane) {
    if (lane == 1) {
        return 2.0;
    }
    if (lane == 2) {
        return 6.0;
    }
    if (lane == 3) {
        return 10.0;
    }
    return 0;
}

// Calculates jerk minimizing path
vector<double> PathPlanner::computeMinimumJerk(vector<double> start, vector<double> end, double max_time, double time_inc) {
    Eigen::MatrixXd A(3, 3);
    Eigen::VectorXd b(3);
    Eigen::VectorXd x(3);

    double t = max_time;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A << t3, t4, t5,
            3 * t2, 4 * t3, 5 * t4,
            6 * t, 12 * t2, 20 * t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
            end[1] - (start[1] + start[2] * t),
            end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    vector<double> result;
    for (double t = time_inc; t < max_time + 0.001; t += time_inc) {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
        result.push_back(r);
    }
    return result;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
PathPlanner::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

}

path_t PathPlanner::JerkPath(projection_t &new_setpoints, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                             vector<double> &map_waypoints_y) {

    // Conditions for minimum jerk in s (zero start/end acceleration)
    double start_s = new_setpoints.start_s;
    double start_speed = new_setpoints.start_speed;
    double end_s = new_setpoints.end_s;
    double end_speed = new_setpoints.end_speed;

    // Conditions for minimum jerk in d (zero start/end acceleration and velocity, indexing by lane)
    double start_d = LaneToD(new_setpoints.src_lane);
    double end_d = LaneToD(new_setpoints.trg_lane);

    // Generate minimum jerk path in Frenet coordinates
    vector<double> next_s_vals = computeMinimumJerk({start_s, start_speed, 0.0},
                                                    {end_s, end_speed, 0.0},
                                                    PATH_PLAN_SECONDS,
                                                    PATH_PLAN_INCREMENT);

    vector<double> next_d_vals = computeMinimumJerk({start_d, 0.0, 0.0},
                                                    {end_d, 0.0, 0.0},
                                                    PATH_PLAN_SECONDS,
                                                    PATH_PLAN_INCREMENT);

    // Convert Frenet coordinates to map coordinates
    vector<double> next_x_vals = {};
    vector<double> next_y_vals = {};

    for (int i = 0; i < next_s_vals.size(); i++) {
        vector<double> xy = getXY(fmod(next_s_vals[i], MAX_TRACK_S),
                                  next_d_vals[i],
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y);
        cout << "x= " << xy[0] << "y= " << xy[1] << endl;
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    path_t path;
    path.path_x = next_x_vals;
    path.path_y = next_y_vals;
    path.last_s = next_s_vals[next_s_vals.size() - 1];
    path.last_d = next_d_vals[next_d_vals.size() - 1];
    cout << "last_s= " << path.last_s << "last_d= " << path.last_d << endl;
    return path;
}
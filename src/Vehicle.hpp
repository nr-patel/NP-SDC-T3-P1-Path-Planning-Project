#pragma once

#include <vector>

using std::vector;

const double MAX_SPEED = 49.5;
const double MAX_ACC = .224;

struct Vehicle {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double end_path_s;
  vector<double> previous_path_x;
  vector<double> previous_path_y;

  int getHistorySize() const { return previous_path_x.size(); }
};

struct Prediction {
  bool own_lane_blocked = false;
  bool left_lane_blocked = false;
  bool right_lane_blocked = false;
  double front_car_velocity = 0.0;
};

struct BehaviorState {
  int lane;
  double acceleration;
  double ref_velocity;

  BehaviorState(int lane, double acc, double ref_vel)
      : lane(lane), acceleration(acc), ref_velocity(ref_vel) {}
};

#pragma once

#include <math.h>
#include <vector>
#include "Path.hpp"
#include "Vehicle.hpp"

#include "spline.h"

using namespace std;

struct RefPoint {
  double x;
  double y;
  double yaw;

  RefPoint(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}

  pair<double, double> localToGlobal(double x, double y) {
    double global_x = x * cos(this->yaw) - y * sin(this->yaw);
    double global_y = x * sin(this->yaw) + y * cos(this->yaw);

    global_x += this->x;
    global_y += this->y;
    return {global_x, global_y};
  }

  pair<double, double> globalToLocal(double x, double y) {
    double shift_x = x - this->x;
    double shift_y = y - this->y;

    double local_x = shift_x * cos(-this->yaw) - shift_y * sin(-this->yaw);
    double local_y = shift_x * sin(-this->yaw) + shift_y * cos(this->yaw);
    return {local_x, local_y};
  }
};

class Trajectory {
  const Vehicle& _car_state;
  const Path& _path;
  const BehaviorState& _maneuver;

  RefPoint _ref_point;
  vector<double> _anchor_pts_x;
  vector<double> _anchor_pts_y;
  tk::spline _spline;
  double _end_velocity;

  vector<double> _eval_xx;
  vector<double> _eval_yy;

  void addAnchorPoint(double x, double y);
  void addAnchorPoint(const vector<double> &pt);
  bool hasPreviousData() const { return _car_state.getHistorySize() > 1; }
  RefPoint getReferencePoint() const;
  vector<double> getPreviousPoint() const;
  void buildSpline();
  void addPreviousDataToResult();
  void addNewDataToResult();

 public:
  Trajectory(const Vehicle &car_state, const Path &path, const BehaviorState &maneuver)
      : _car_state(car_state), _path(path), _maneuver(maneuver), _ref_point(getReferencePoint()) {}

  void calculate();


  vector<double> xx() const { return _eval_xx; }
  vector<double> yy() const { return _eval_yy; }
  double end_velocity() const { return _end_velocity; }
};

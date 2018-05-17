#include <algorithm>

#include "Trajectory.hpp"
#include "util.h"
#include "spline.h"

using std::min;

double laneToFrenet(int lane) { return 2 + 4 * lane; }

void Trajectory::addAnchorPoint(double x, double y) {
  auto local = _ref_point.globalToLocal(x, y);
  _anchor_pts_x.push_back(local.first);
  _anchor_pts_y.push_back(local.second);
}

void Trajectory::addAnchorPoint(const vector<double> &pt) {
  addAnchorPoint(pt[0], pt[1]);
}

RefPoint Trajectory::getReferencePoint() const {
  if (hasPreviousData()) {
    int prev_size = _car_state.previous_path_x.size();
    double x = _car_state.previous_path_x[prev_size - 1];
    double y = _car_state.previous_path_y[prev_size - 1];

    double x_prev = _car_state.previous_path_x[prev_size - 2];
    double y_prev = _car_state.previous_path_y[prev_size - 2];
    double yaw = atan2(y - y_prev, x - x_prev);
    return RefPoint(x, y, yaw);
  } else {
    return RefPoint(_car_state.x, _car_state.y, deg2rad(_car_state.yaw));
  }
}

void Trajectory::buildSpline() {
  _spline.set_points(_anchor_pts_x, _anchor_pts_y);
}

vector<double> Trajectory::getPreviousPoint() const {
  double x_prev, y_prev;
  if (hasPreviousData()) {
    int prev_size = _car_state.previous_path_x.size();
    x_prev = _car_state.previous_path_x[prev_size - 2];
    y_prev = _car_state.previous_path_y[prev_size - 2];
  } else {
    x_prev = _car_state.x - cos(_car_state.yaw);
    y_prev = _car_state.y - sin(_car_state.yaw);
  }
  return {x_prev, y_prev};
}

void Trajectory::addPreviousDataToResult() {
  for (int i = 0; i < _car_state.getHistorySize(); i++) {
    _eval_xx.push_back(_car_state.previous_path_x[i]);
    _eval_yy.push_back(_car_state.previous_path_y[i]);
  }
}

void Trajectory::addNewDataToResult() {
  double target_x = 30.0;
  double target_y = _spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;
  _end_velocity = _maneuver.ref_velocity;
  for (int i = 1; i < 50 - _car_state.getHistorySize(); i++) {
    _end_velocity = min(MAX_SPEED, _end_velocity + _maneuver.acceleration);
    double N = target_dist / (0.02 * _end_velocity / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = _spline(x_point);

    x_add_on = x_point;

    auto global = _ref_point.localToGlobal(x_point, y_point);
    _eval_xx.push_back(global.first);
    _eval_yy.push_back(global.second);
  }
}

void Trajectory::calculate() {
  vector<double> prev_point = getPreviousPoint();

  addAnchorPoint(prev_point);
  addAnchorPoint(_ref_point.x, _ref_point.y);

  double lane_d = laneToFrenet(_maneuver.lane);
  addAnchorPoint(_path.getXY(_car_state.s + 30, lane_d));
  addAnchorPoint(_path.getXY(_car_state.s + 60, lane_d));
  addAnchorPoint(_path.getXY(_car_state.s + 90, lane_d));

  buildSpline();

  addPreviousDataToResult();
  addNewDataToResult();
}
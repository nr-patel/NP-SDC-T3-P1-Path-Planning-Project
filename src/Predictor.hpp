#pragma once

#include <math.h>
#include <vector>
#include "Vehicle.hpp"

using std::vector;

class SensedCar {
  const vector<double> &_readings;

 public:
  SensedCar(const vector<double> &readings) : _readings(readings) {}
  int lane() const {
    double d = _readings[6];
    if (d > 0 && d < 4) {
      return 0;
    } else if (d > 4 && d < 8) {
      return 1;
    } else if (d > 8 && d < 12) {
      return 2;
    }
    return -1;
  }

  double speed() const {
    double vx = _readings[3];
    double vy = _readings[4];
    return sqrt(vx * vx + vy * vy);
  }

  double s_estimated(double dt) const {
    double s = _readings[5];
    return s + speed() * dt;
  }
};

class Predictor {
  const Vehicle &_ego_car;
  int _ego_lane;
  void analyzeCarPosition(const SensedCar &car, Prediction &pred) const;

 public:
  Predictor(const Vehicle &ego_car, int ego_lane)
      : _ego_car(ego_car), _ego_lane(ego_lane) {}
  Prediction calculate(const vector<vector<double>> &sensor_fusion);
};

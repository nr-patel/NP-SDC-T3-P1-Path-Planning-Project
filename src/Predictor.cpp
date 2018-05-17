#include "Predictor.hpp"
#include <math.h>

void Predictor::analyzeCarPosition(const SensedCar &other_car,
                                   Prediction &pred) const {
  int car_lane = other_car.lane();
  if (car_lane < 0) {
    return;
  }

  double other_car_s = other_car.s_estimated(0.02 * _ego_car.getHistorySize());
  bool is_close_enough =  fabs(_ego_car.s - other_car_s) < 30;

  if (car_lane == _ego_lane) {
    if (is_close_enough && (other_car_s > _ego_car.s)) {
      pred.own_lane_blocked = true;
      pred.front_car_velocity = other_car.speed() * 2.24;
    }
  } else if (car_lane == (_ego_lane - 1)) {
    // Car left
    pred.left_lane_blocked |= is_close_enough;
  } else if (car_lane == (_ego_lane + 1)) {
    pred.right_lane_blocked |= is_close_enough;
  }
}

Prediction Predictor::calculate(const vector<vector<double>> &sensor_fusion) {
  Prediction result;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    SensedCar other_car(sensor_fusion[i]);
    analyzeCarPosition(other_car, result);
  }
  return result;
}

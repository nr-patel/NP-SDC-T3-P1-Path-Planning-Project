#pragma once

#include "Vehicle.hpp"

class Behavior {
  const Prediction &_prediction;

  bool isMyLaneBlocked() const { return _prediction.own_lane_blocked; }
  bool canChangeLeft(int cur_lane) const {
    return (cur_lane > 0) && !_prediction.left_lane_blocked;
  }
  bool canChangeRight(int cur_lane) const {
    return (cur_lane < 2) && !_prediction.right_lane_blocked;
  }

  bool canGoBackToCenter(int cur_lane) {
    return (cur_lane == 0 && !_prediction.right_lane_blocked) ||
           (cur_lane == 2 && !_prediction.left_lane_blocked);
  }

 public:
  Behavior(const Prediction &prediction) : _prediction(prediction) {}
  void update(BehaviorState& state);
};

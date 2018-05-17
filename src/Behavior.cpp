#include "Behavior.hpp"
#include <iostream>

using std::cout;
using std::endl;

int sign(double x) {
  if (x < 0) return -1;
  if (x > 0) return 1;
  return 0;
}

void Behavior::update(BehaviorState& state) {
  cout << " MY: " << _prediction.own_lane_blocked
       << " LEFT: " << _prediction.left_lane_blocked
       << " RIGHT: " << _prediction.right_lane_blocked << endl;

  state.acceleration = 0.0;
  if (isMyLaneBlocked()) {
    cout << "BLOCKED ";
    if (canChangeLeft(state.lane)) {
      cout << "- LEFT ";
      state.lane--;
    } else if (canChangeRight(state.lane)) {
      cout << "- RIGHT ";
      state.lane++;
    } else {
      cout << "- FOLLOW " << _prediction.front_car_velocity << " ";
      if (_prediction.front_car_velocity <= state.ref_velocity) {
        state.acceleration = -MAX_ACC;
      }
    }
  } else {
    if (state.lane != 1) {
      if (canGoBackToCenter(state.lane)) {
        cout << "- CENTER ";
        state.lane = 1;
      }
    }
    if (state.ref_velocity < MAX_SPEED) {
      cout << "- CURRENT ";
      state.acceleration = MAX_ACC;
    }
  }

  if (state.acceleration > 0) {
    cout << "ACCELERATE";
  } else if (state.acceleration < 0) {
    cout << "DECELERATE";
  }
  cout << endl;
}
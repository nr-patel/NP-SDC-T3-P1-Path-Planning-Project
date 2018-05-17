#pragma once

#include <vector>

using std::vector;

class Path {
 public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  void addWaypoint(double x, double y, double s, double d_x, double d_y);
  vector<double> getXY(double s, double d) const;
};


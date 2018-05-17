#include "Path.hpp"
#include "util.h"

void Path::addWaypoint(double x, double y, double s, double d_x, double d_y) {
  map_waypoints_x.push_back(x);
  map_waypoints_y.push_back(y);
  map_waypoints_s.push_back(s);
  map_waypoints_dx.push_back(d_x);
  map_waypoints_dy.push_back(d_y);
}

vector<double> Path::getXY(double s, double d) const {
  return ::getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
}
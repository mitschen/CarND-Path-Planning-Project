/*
 * constansts.hpp
 *
 *  Created on: 20.01.2018
 *      Author: micha
 */

#include <vector>
#include <limits>


double const c_maxSpeed = 49.5; //mph
int const c_noLanes = 3;
int const c_laneSize = 4; //4 meter lanes
int const c_halfALane(c_laneSize/2);
double const c_mph2mps_factor = 1./2.24;
int const c_noProjectionPoints = 50; //how many points should be projected
int const c_projectiongTime = 1; //in which time in seconds
double const c_timeSpanPerProjectionPoint = c_projectiongTime / (double) c_noProjectionPoints;

int const c_numberOfPredictions = 30;
double const c_horizont_s = 5.; //take 5 seconds of future into considerations


struct SCarPos
{
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  int car_lane;
};

struct SPathData
{
  std::vector<double> prev_path_x;
  std::vector<double> prev_path_y;
  double end_path_s;
  double end_path_d;
};

struct SMapWaypoint
{
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
};


struct SWaypoints
{
  std::vector<double> x;
  std::vector<double> y;
};

struct SVehicle
{
  SVehicle(double const defaultPosInTime) : index(-1), velocity(0.), distanceToReference(std::numeric_limits<double>::max()), positionInTime{defaultPosInTime}
  {};
  int index;
  double velocity;
  double distanceToReference; //distance to my car
  double positionInTime[c_numberOfPredictions];
};

struct SLane
{
  SLane() : lane(-1), predecessor(-100000.), successor(100000.), velocityExpected(0.), velocityDevelopment(c_noProjectionPoints, 0.){};
  int lane;
  SVehicle predecessor;
  SVehicle successor;
  double velocityExpected;
  std::vector<double> velocityDevelopment;
};


//methods available in main.cpp
/**
 * calculate the euclidian distance
 */
double distance(double x1, double y1, double x2, double y2);
/**
 * identifies the index of the closest waypoint from x,y coordinates
 * @please note: closest waypoint might be behind x,y!
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
/**
 * returns index of the next waypoint (in front of x,y)
 */
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * returns a vector{s, d} representing x, y in frenet coordinates
 */
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
/**
 * returns a vector{x,y} which represents the frenet coordinates s,y in cartesian
 */
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<SLane> prepareLanes(SCarPos const &car, std::vector< std::vector<double> > const &sensor);

bool laneChangeAllowed(SCarPos const &car, SLane const&);

void speedvalueDevelopmentOnLane(SCarPos const &car, SLane &lane, double const speedDiff = 5.224 );

/**
 * returns the next x,y waypoints for our car
 */
SWaypoints calcNextXY( SCarPos const &, SPathData const &, SMapWaypoint const &, double const &velocity_mps, int const &currentLane);

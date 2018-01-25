/*
 * constansts.hpp
 *
 *  Created on: 20.01.2018
 *      Author: micha
 */

#include <vector>
#include <limits>


double const c_maxSpeed = 50.;          //max speed in mph
double const c_maxAcceleration = .224;  //max acceleration mph
double const c_updateTime = 0.02;       //20ms delay between two update cycles
int const c_noLanes = 3;                //total number of lanes
int const c_laneSize = 4;               //lanesize in meter
int const c_halfALane(c_laneSize/2);    //const for half a lane
double const c_mph2mps_factor = 1./2.24;//factor to convert mph in mps
int const c_noProjectionPoints = 50;    //how many points should be projected (for waypoints)
int const c_projectiongTime = 1;        //in which time in seconds
//Please note:  the combination of 50 points with a projection time of 1 seconds reflects the
//              update time cycle given by simulator (20ms)
double const c_timeSpanPerProjectionPoint = c_projectiongTime / (double) c_noProjectionPoints;

int const c_numberOfPredictions = 9;  //number of prediction points within the c_horizont_s time
double const c_horizont_s = 3.;       //time horizont in seconds we're predicting the car when chosing lanes
int const c_tangentialJerkBuffer = 20; //we must stay at least for 10 iterations in case of lanechange attempts passing two lanes
                                       //see SState::laneChangeBuffer for further details

/**
 * CarPosition struct given from infrastructor
 */
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

/**
 * path data given from infrastructure
 */
struct SPathData
{
  std::vector<double> prev_path_x;
  std::vector<double> prev_path_y;
  double end_path_s;
  double end_path_d;
};

/**
 * waypoint struct given from infrastructure
 */
struct SMapWaypoint
{
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
};

/**
 * struct containing a list of waypoints
 */
struct SWaypoints
{
  std::vector<double> x;
  std::vector<double> y;
};

/**
 * struct used to represent one of the other vehicles
 * This structure is used to calculate the vehicle position over time
 * by considering the velocity of the vehicle. Each object contains
 * a distanceToReference member which represents the distance of the
 * vehicle to my projected car position
 */
struct SVehicle
{
  SVehicle(double const defaultPosInTime) : index(-1), velocity(0.), distanceToReference(std::numeric_limits<double>::max()), positionInTime{defaultPosInTime}
  {};
  int index;                      //car index (of sensor fusion)
  double velocity;                //velocity of the car in mph
  double distanceToReference;     //absolute distance to my projected car position
  double positionInTime[c_numberOfPredictions]; //position of the vehicle over time
};

/**
 * struct represents a certain lane and the predecessor/ successor cars on
 * the lane with reference to my car. After calling @see speedvalueDevelopmentOnLane
 * the values velocityExpected as well as preferenceVal are filled.
 * Our decision to chose the lane depends on the preferenceVal. The velocity
 * we're allowed has a direct impact on our projection
 *
 */
struct SLane
{
  SLane() : lane(-1), predecessor(-100000.), successor(100000.), velocityExpected(0.), preferenceVal(1.){};
  int lane;                 //lane number(0..2)
  SVehicle predecessor;     //the car in my back
  SVehicle successor;       //the car in front of me
  double velocityExpected;  //the velocity i'm assuming i can drive on that lane
  double preferenceVal;     //the preference value (how promising) of the lane (based on the velocity on the lane projected in time)
};

/**
 * the car state represents the projected state of our car
 */
struct SState
{
  int     currentLane;        //which lane our projection is currently in
  int     intendedLane;       //which lane we want to reach (used only if we want to cross two lanes)
  double  currentSpeed;       //speed of our projection
  int     laneChangeBuffer;   //a simple counter used in case when crossing two lanes. The laneChangeBuffer
                              //is set if we want to cross two lanes and will be reduced every
                              //update cycle. This counter guarantees that we're staying at
                              //least for a moment in the middle lane (the one we want to cross)
                              //before moving to our intended one
};

struct SForecastState
{
  int     offset;           //points in front we're forecasting (path_data.prev_path.x.size())
  int     middleOfLane;     //middle of our lane in meter
  double  car_s;            //car_s projection
  double  car_d;            //car_d projection
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

/**
 * identify the lanenumber for a d value
 */
int identifyLane(double const d);

/**
 * use the forecast position of the car as well as the sensorfusion data
 * to identify for each lane the predecessor/ successor to us
 */
std::vector<SLane> prepareLanes(SForecastState const &car, std::vector< std::vector<double> > const &sensor);

/**
 * verify if by given car_speed and given projected position we are allowed to change
 * to a certain lane
 */
bool laneChangeAllowed(double const &car_speed, SForecastState const &car, SLane const&, bool passing = false, bool dump=false);

/**
 * calculate a projection of the car-speed/ position on a certain lane, consider
 * predecessor/ successors. Please note: prepareLanes must be called before.
 * The return value represents the preferenceValue (higher is better)
 */
double speedvalueDevelopmentOnLane(double const &car_speed, SForecastState const &car, SLane &lane, bool passing = false, double const speedDiff = c_maxAcceleration );

/**
 * calculate a security distance to predecessor/ successor based on speed (meter per second)
 * In case of passing = true (we want to change lanes in order to pass a successor)
 * we're reducing temporarly the security distance
 */
double getSecurityBelt(double const &speed_mps, bool passing = false);

/**
 * returns the next, projected x,y waypoints for our car - the SWaypoints
 * contains the previous path information plus the projection of the car position
 * in future. The total number of waypoints is equal to c_noProjectionPoints
 *
 * With the current setup we're project our car 1 second to future
 */
SWaypoints calcNextXY( SCarPos const &, SPathData const &, SMapWaypoint const &, double const &velocity_mps, int const &currentLane);

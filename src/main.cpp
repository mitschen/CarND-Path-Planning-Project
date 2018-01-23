#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//refere http://kluge.in-chemnitz.de/opensource/spline/
#include "spline.h"
#include "limits"
#include "constants.hpp"
#include <cstring>
using namespace std;

// for convenience
using json = nlohmann::json;



// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//Might be the closest one, but this one might be behind the car
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}
//Represents the next waypoint in front of the car
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//MScharf: Use it as a blackbox
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
//MScharf: Invers calculation from Fenet
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

std::vector<int> getLaneStartPos()
{
  vector<int> lanestart(c_noLanes);
  for(int i(0); i<c_noLanes; ++i)
  {
    lanestart[i] = i*c_laneSize;
  }
  return lanestart;
}

int identifyLane(double const d)
{
  static const vector<int> c_laneStart(getLaneStartPos());
  int resultingLane(1);
  for(int _maxI(c_laneStart.size()); resultingLane<_maxI; ++resultingLane)
  {
    if(d<c_laneStart[resultingLane])
      break;
  }
  return (resultingLane-1);
}


std::vector<SLane> prepareLanes(SForecastState const &car, std::vector< std::vector<double> > const &sensor)
{
  //index in sensorfusion array
  int const cid(0);
  int const cx(1);
  int const cy(2);
  int const cvx(3);
  int const cvy(4);
  int const cs(5);
  int const cd(6);

  std::vector<SLane> lanes;
  lanes.resize(c_noLanes);
  for(int i(0); i<c_noLanes; ++i)
  {
    lanes[i].lane=i;
  }

  //identify for each lane the closest predecessor and the successor
  for(int i(0), _maxI(sensor.size()); i<_maxI; ++i)
  {
    //assuming that the car stays in its lane
    SLane &affectedLane(lanes[identifyLane(sensor[i][cd])]);
    double const sensor_speed(sqrt(sensor[i][cvx]*sensor[i][cvx] + sensor[i][cvy]*sensor[i][cvy]));
    //where is the car at the time of our forecast
    double const sensor_s(sensor[i][cs] + sensor_speed * car.offset * c_timeSpanPerProjectionPoint);
    double distance(sensor_s-car.car_s);
    if( (distance < 0.) && (affectedLane.predecessor.distanceToReference>fabs(distance)))
    {
      affectedLane.predecessor.distanceToReference = fabs(distance);
      affectedLane.predecessor.index = i;
      affectedLane.predecessor.velocity = sensor_speed;
      affectedLane.predecessor.positionInTime[0] = sensor_s;
    }
    else if( (distance> 0.) && (affectedLane.successor.distanceToReference>distance))
    {
      affectedLane.successor.distanceToReference = distance;
      affectedLane.successor.index = i;
      affectedLane.successor.velocity =  sensor_speed;
      affectedLane.successor.positionInTime[0] = sensor_s;
    }
  }

  //now calculate the projection of each predecessor and successor on each lane
  for(int i(0); i<c_noLanes; ++i)
  {
    SLane &curLane(lanes[i]);
    SVehicle &pred(lanes[i].predecessor);
    double predDeltaS(c_horizont_s / c_numberOfPredictions * pred.velocity);
    SVehicle &succ(lanes[i].successor);
    double succDeltaS(c_horizont_s / c_numberOfPredictions * succ.velocity);
    //no predecessor/ successor
    predDeltaS = pred.velocity==0.?1.:predDeltaS;
    succDeltaS = succ.velocity==0.?1.:succDeltaS;
    for(int j(1); j<c_numberOfPredictions; ++j)
    {
      pred.positionInTime[j] = pred.positionInTime[j-1]+predDeltaS;
      succ.positionInTime[j] = succ.positionInTime[j-1]+succDeltaS;
    }
  }
  return lanes;
}


bool laneChangeAllowed(double const &car_speed, SForecastState const &car, SLane const&lane, bool passing, bool dump)
{
  bool retVal(true);
  double const securityBelt(getSecurityBelt(car_speed*c_mph2mps_factor, passing)); //according to german law: 1/2 of km/h in meters
  if(dump)
    cout<<"Lane "<<lane.lane<<" Belt "<<securityBelt<<" Pred "<<lane.predecessor.distanceToReference << " Succ "<<lane.successor.distanceToReference<<endl;
  retVal = retVal && lane.predecessor.distanceToReference >= (securityBelt/4.);
  retVal = retVal && lane.successor.distanceToReference >= securityBelt;

//  //verify the future for at least one second
//  int stepsForOneSecond( (int)c_numberOfPredictions / c_horizont_s);
//  double carPos(car.car_s*c_mph2mps_factor);
//  double deltaS(car.car_speed / (double) stepsForOneSecond);
//  for(int i(0); retVal && (i<stepsForOneSecond); i++)
//  {
//    retVal = retVal && fabs(lane.predecessor.positionInTime[i]-carPos)>=(securityBelt/4.);
//    retVal = retVal && fabs(lane.successor.positionInTime[i]-carPos)>=(securityBelt);
//    carPos += deltaS;
//  }
  return retVal;
}

double speedvalueDevelopmentOnLane(double const &car_speed, SForecastState const &car, SLane &lane,  bool passing, double const speedDiff)
{

  double &avg_velocity(lane.velocityExpected);
  double &preferenceVal(lane.preferenceVal);
  preferenceVal = 0.;
  double const c_maxSpeedmps(c_maxSpeed );

  double currentCarPos(car.car_s);
  double currentCarSpeed(car_speed);
//  double accelaration(speedDiff * c_updateTime);
//  double accDistanceFraction(0.5 * accelaration * pow(c_horizont_s / (double)c_numberOfPredictions,2));
//  double speedDiffFraction(accelaration * (c_horizont_s / (double)c_numberOfPredictions));
//  double const securityBeltDelta(getSecurityBelt(speedDiffFraction));
  double const timeDuringStep(c_horizont_s / (double) c_numberOfPredictions);
  double const diff_s(speedDiff * timeDuringStep);
  for(int i(0); i < c_numberOfPredictions; ++i)
  {

    double const securityBelt(getSecurityBelt(currentCarSpeed*c_mph2mps_factor)); //according to german law: 1/2 of km/h in meters
    if( ( (currentCarPos + securityBelt + diff_s) < lane.successor.positionInTime[i])
      && ((currentCarSpeed + speedDiff) < c_maxSpeedmps) )
    {
      //increase the speed
      currentCarSpeed+=speedDiff;
//      currentCarPos+=diff_s;
    }
    else if((currentCarPos + securityBelt) < lane.successor.positionInTime[i])
    {
      //keep the speed
    }
    else
    {
      //reduce the speed
      currentCarSpeed-=speedDiff;
//      currentCarPos-=diff_s;
    }
    currentCarPos+=(currentCarSpeed*timeDuringStep);
    preferenceVal+=currentCarSpeed;
    //reduce the impact of speed for distance
//    speedDiffFraction *= .9;
    if(i == 0)
    {
      avg_velocity = currentCarSpeed;
    }
  }
//  avg_velocity = currentCarSpeed +  (currentCarSpeed - car_speed)/ (double) c_numberOfPredictions;
//  cout<<"Avg_velocity "<<avg_velocity<<" CarSpeed " <<car_speed<<endl;
  return preferenceVal;
}

double getSecurityBelt(double const &speed_mps, bool passing)
{
  //if we want to pass a car, we reduce the security belt to 5 m
  if(passing)
    return 7;
  return speed_mps * 3.6 / 3.0;
}

/**
 * get the cost function for lateral changes with respect to longitudinal
 * distance d<->s. This implicit will represent jerk. If we're doing
 * big lateral changes in a very short longitudinal scretch, this will
 * result in high jerk
 * @see BahviourPlanning chapter 12
 */
double costFunction_Jerk(vector<double> const &_s, vector<double> const &_d)
{
  double const &goalD(_d.back());
  double const &goalS(_s.back());
  double costVal(0.);
  for(int i(0), _maxI(_s.size()-1); i<_maxI; ++i)
  {
    double const &d(_d[i]), &s(_s[i]);
    double const deltaD( fabs(goalD - d));
    costVal+= (1 - exp(-(deltaD/(goalS-s) )));
  }
  return costVal / (double) (_s.size()-1);
}


/**
 * get the cost of a velocity trajectory normalized between 0-1
 * @see BahviourPlanning chapter 11
 */
double costFunction_Velocity(vector<double> const &velocity)
{
  static double STOP_COST(0.8);
  static double SPEED_LIMIT(50.);
  static double BUFFER_V(0.5);
  static double TARGET_SPEED(SPEED_LIMIT-BUFFER_V);
  double costValue(0.);
  for(int i(0), _maxI(velocity.size()); i<_maxI; ++i)
  {
    double const &cur_vel(velocity[i]);
    if(cur_vel>SPEED_LIMIT)
    {
      costValue+=1.;
    }
    else if(cur_vel<=TARGET_SPEED)
    {
      costValue += STOP_COST*( (TARGET_SPEED-cur_vel) / TARGET_SPEED);
    }
    else
    {
      costValue+=(cur_vel-TARGET_SPEED) / BUFFER_V;
    }

  }
  return costValue / (double)(velocity.size());
}


SWaypoints calcNextXY( SCarPos const &car, SPathData const &path, SMapWaypoint const &map, double const &velocity_mps, int const &currentLane)
{
  SWaypoints resultingWaypoints;
  resultingWaypoints.x.clear();
  resultingWaypoints.y.clear();

  //MScharf: member matching to reuse existing listing from AAron
  double const &car_x(car.car_x);
  double const &car_y(car.car_y);
  double const &car_yaw(car.car_yaw);
  //          //if the is a previous path, we reset the car's position
  //          //to the previous path last s position
  //          if(prev_size > 0)
  //          {
  //            car_data.car_s = path_data.end_path_s;
  //          }
  double const &car_s(path.prev_path_x.size()>0?path.end_path_s:car.car_s);
  vector<double> const &previous_path_x(path.prev_path_x);
  vector<double> const &previous_path_y(path.prev_path_y);
  vector<double> const &map_waypoints_s(map.map_waypoints_s);
  vector<double> const &map_waypoints_x(map.map_waypoints_x);
  vector<double> const &map_waypoints_y(map.map_waypoints_y);

  int const c_prevPathSize(previous_path_x.size());
  int const c_middleOfCurrentLane(c_laneSize * currentLane + c_laneSize/2);

  //define the actual (x,y) points we will use for the planner
  vector<double> &next_x_vals(resultingWaypoints.x);
  vector<double> &next_y_vals(resultingWaypoints.y);



  //Arons changes
  //create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  //later we will interpolate these waypoints with a spline and fill it
  //in with more points that control speed
  vector<double> ptsx, ptsy;

  //reference x,y, yaw states
  //either we will reference the starting point as where the car is or
  //at the previous paths end point
  double ref_x(car_x);
  double ref_y(car_y);
  double ref_yaw(deg2rad(car_yaw));

  //if previous size is almost empty, use the car as starting reference
  if(c_prevPathSize < 2)
  {
    //use the two points that make the path tangent to the car
    double prev_car_x(car_x-cos(car_yaw));
    double prev_car_y(car_y-sin(car_yaw));

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  //use the previous path's end point as starting reference
  else
  {
    //redefine reference state as previous path end point
    ref_x = previous_path_x.back();
    ref_y = previous_path_y.back();
    double ref_x_prev(previous_path_x[previous_path_x.size()-2]);
    double ref_y_prev(previous_path_y[previous_path_y.size()-2]);
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    //use the two points that make the path tangent to the car
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  //in frenet add evenly 30m spaced points ahead of the starting reference

  vector<double> next_wp0(getXY(car_s+1*30,c_middleOfCurrentLane, map_waypoints_s, map_waypoints_x, map_waypoints_y));
  vector<double> next_wp1(getXY(car_s+2*30,c_middleOfCurrentLane, map_waypoints_s, map_waypoints_x, map_waypoints_y));
  vector<double> next_wp2(getXY(car_s+3*30,c_middleOfCurrentLane, map_waypoints_s, map_waypoints_x, map_waypoints_y));

  //containing two previous points and the projection for 30,60,90 meter
  ptsx.push_back(next_wp0.front());
  ptsy.push_back(next_wp0.back());
  ptsx.push_back(next_wp1.front());
  ptsy.push_back(next_wp1.back());
  ptsx.push_back(next_wp2.front());
  ptsy.push_back(next_wp2.back());

  //we are moving/ shifting the coordinates in the way, that the first coordinate
  //referes to our origin of the next coordinate system
  for(int i(0), _maxI(ptsx.size()); i < _maxI; i++)
  {
    double shift_x(ptsx[i]-ref_x);
    double shift_y(ptsy[i]-ref_y);

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  //create a spline
  tk::spline s;

  //set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  //start with all the previous path points from last time
  for(int i(0), _maxI(previous_path_x.size()); i < _maxI; ++i)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  //Calculate how to break up the spline points so that we travel at
  //our desired reference velocity
  double target_x(30.); //30 meters along x
  //use the spline s to identify the y coordinate on spline for target_x value
  double target_y(s(target_x));
  //target distance
  double target_dist(sqrt((target_x)*(target_x)+(target_y)*(target_y)));

  double x_add_on(0.); //we're starting at the origin - so no x-addon

  //fill up the rest of our path planner after filling it with previous
  //points, here we will always output 50 points = c_noProjectionPoints
  for(int i(1),_maxI(c_noProjectionPoints-previous_path_x.size());i<=_maxI ; ++i)
  {
    //transform mph into meter per step
    //mph into m/s is ref_vel/2.24
    //But we're dealing with 50 steps per seconds - that means
    //the projection step is 0.02
    //note: velocity is already meter per second
    double ref_vel_ms(c_timeSpanPerProjectionPoint*velocity_mps);
    double N(target_dist/ref_vel_ms);
    double x_point(x_add_on+(target_x)/N);
    double y_point(s(x_point)); //use the spline again to fetch the y coord

    //set x_add_on to the current x point for next iteration
    x_add_on = x_point;

    double x_ref(x_point), y_ref(y_point);
    //rotate back to normal after rotating it earlier
    //so from local coordinates (car) to the global coordinates
    x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point+=ref_x;
    y_point+=ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);


  }
  return resultingWaypoints;
}

void dumpSensorFusion(std::vector<double> &v)
{
  cout << "SensorFusion "<<v[0]<< " (x,y) = "<<v[1]<<","<<v[2]<<" (vx,vy) ="<<v[3]<<","<<v[4]<<" (s,d) "<<v[5]<<","<<v[6]<<endl;
}

int main() {
  using namespace std;
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);


  SMapWaypoint map_data;

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_data.map_waypoints_x.push_back(x);
  	map_data.map_waypoints_y.push_back(y);
  	map_data.map_waypoints_s.push_back(s);
  	map_data.map_waypoints_dx.push_back(d_x);
  	map_data.map_waypoints_dy.push_back(d_y);
  }

  //Some variables
  SState myCarState;
  myCarState.currentLane = 1;
  myCarState.intendedLane = 1;
  myCarState.currentSpeed = 0.;
  myCarState.laneChangeBuffer = 0;
//  int current_lane(1); //of a total of 3 [0,1,2]
//  double ref_vel(0); //mphs


  h.onMessage([&myCarState, &map_data](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        SCarPos car_data;
        SPathData path_data;

        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          car_data.car_x = j[1]["x"];
          car_data.car_y = j[1]["y"];
          car_data.car_s = j[1]["s"];
          car_data.car_d = j[1]["d"];
          car_data.car_yaw = j[1]["yaw"];
          car_data.car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          std::vector<double> prev_x = (j[1]["previous_path_x"]);
          std::vector<double> prev_y = (j[1]["previous_path_y"]);
          path_data.prev_path_x = prev_x;
          path_data.prev_path_y = prev_y;
          	// Previous path's end s and d values 
          path_data.end_path_s = j[1]["end_path_s"];
          path_data.end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion_raw = j[1]["sensor_fusion"];
          vector< vector<double > > sensor_fusion;

          //filter sensor values to the ones in my surrounding
          for(int i(0), _maxI(sensor_fusion_raw.size()); i< _maxI; ++i)
          {
            if(fabs(car_data.car_s - (double) sensor_fusion_raw[i][5])>1000.)
            {
              //noop
            }
            else
            {
              sensor_fusion.push_back(sensor_fusion_raw[i]);
            }
          }


          json msgJson;

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
          //The previous path size
//          myCarState.currentLane = identifyLane(car_data.car_d);
          SForecastState forecast;
          forecast.offset = path_data.prev_path_x.size();
          forecast.middleOfLane = (c_laneSize * myCarState.currentLane + c_laneSize / 2);
          if(forecast.offset>0)
          {
            forecast.car_s = path_data.end_path_s;
            forecast.car_d = path_data.end_path_d;
          }
          else
          {
            forecast.car_s = car_data.car_s;
            forecast.car_d = car_data.car_d;
          }

          auto lanes(prepareLanes(forecast, sensor_fusion));
          bool passing(myCarState.currentLane != myCarState.intendedLane);
          std::pair<bool, double> validLanes[c_noLanes];
          std::pair<int, double> preferredLane {myCarState.currentLane, speedvalueDevelopmentOnLane(myCarState.currentSpeed, forecast, lanes[myCarState.currentLane], passing)};
          int previousLane(myCarState.currentLane);
          bool dump(false);
          for(int i(c_noLanes-1); i>=0; --i)
          {
            validLanes[i].first = (laneChangeAllowed(myCarState.currentSpeed, forecast, lanes[i], passing, dump) || (lanes[i].lane == myCarState.currentLane) );
            validLanes[i].second = validLanes[i].first?speedvalueDevelopmentOnLane(myCarState.currentSpeed, forecast, lanes[i], passing):0.;
            preferredLane = (preferredLane.second<validLanes[i].second) && (validLanes[i].first)?std::pair<int, double>{i, validLanes[i].second}:preferredLane;
          }
          if( (0<--myCarState.laneChangeBuffer) || ((preferredLane.first == myCarState.currentLane) && validLanes[myCarState.currentLane].first) )
          {
            //stay with the previous decision, adjust speed only
            myCarState.currentSpeed = lanes[myCarState.currentLane].velocityExpected;
          }
          else
          {
            //trigger lane switch
            int laneDiff(preferredLane.first - myCarState.currentLane);
            if(abs(laneDiff) <= 1)
            {
              //laneswitch
              myCarState.currentLane = preferredLane.first;
              myCarState.intendedLane = myCarState.currentLane;
              myCarState.currentSpeed = lanes[preferredLane.first].velocityExpected;
            }
            else if ((laneDiff<0) && (validLanes[myCarState.currentLane-1].first))
            {
              //laneswitch down
              myCarState.currentLane = myCarState.currentLane-1;
              myCarState.intendedLane = preferredLane.first;
              myCarState.currentSpeed = lanes[myCarState.currentLane-1].velocityExpected;
              myCarState.laneChangeBuffer = 10;
              //for tough lane changes, make sure to not violate acceleration
              myCarState.currentSpeed -= myCarState.currentSpeed>49.9?0.1:0.;
            }
            else if ((laneDiff>1) && (validLanes[myCarState.currentLane+1].first))
            {
              myCarState.currentLane = myCarState.currentLane+1;
              myCarState.intendedLane = preferredLane.first;
              myCarState.currentSpeed = lanes[myCarState.currentLane+1].velocityExpected;
              myCarState.currentSpeed -= myCarState.currentSpeed>49.9?0.1:0.;
              myCarState.laneChangeBuffer = 10;
            }
            else
            {
              //keep lane
              myCarState.currentSpeed = lanes[myCarState.currentLane].velocityExpected;
              myCarState.intendedLane = preferredLane.first;
            }
          }


          //Some informations written to console
          static const char lanename[c_noLanes] = {'l', 'm', 'r'};
          for(int i(c_noLanes-1); i>=0; --i)
          {
            cout << (i==myCarState.currentLane?"*":i==myCarState.intendedLane?">":" ")<< lanename[i] <<" Lane "<<(validLanes[i].first?" valid  ":"invalid ")<<"Preference "<<setw(7)<<validLanes[i].second<<" ExpectedSpeed "<<lanes[i].velocityExpected<<endl;
          }
          cout << "CarSpeed "<<setw(8)<<car_data.car_speed << " Current "<<setw(8)<< myCarState.currentSpeed<<endl;

          //calculate the projected position in 1 seconds future (according to the Q&A of AAron and DSilver
          SWaypoints nextWayPoints(calcNextXY(car_data, path_data, map_data, (myCarState.currentSpeed*c_mph2mps_factor), myCarState.currentLane));


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = nextWayPoints.x;
          msgJson["next_y"] = nextWayPoints.y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

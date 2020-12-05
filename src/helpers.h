#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


// return the Lane number based on the d value
int getCurrentLane(double &car_d)
{
  if ((car_d >= 8) && (car_d < 12))
  {
    return 2;
  }
  else if ((car_d >= 4) && (car_d < 8))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
// This function will provide the list of lanes where car can switch next
vector<int> getProspectiveLanesToChange(int &lane)
{
  vector<int> lanes;
  for(int i=-1; i < 2; i++)
  {
    int new_lane = lane + i;
    if ((new_lane != lane) && (new_lane >= 0) && (new_lane <= 2))
    {
      lanes.push_back(new_lane);
    }
  }
  return lanes;
}

// This function determines which is the best lane for this given scenario to take
vector<int> getBestLaneToChange(const vector<int> &next_lanes, const vector<vector<double>> &sensor_fusion,
                                const double &car_s, const int &previous_size)
{
  int next_lane_size = next_lanes.size();
  vector<int> best_lanes(next_lane_size, 0);
  for(int i =0; i < next_lane_size; i++)
  {
    int current_lane = next_lanes[i];
    // iterate over all potential future lanes
    for(int j = 0; j < sensor_fusion.size(); j++)
    {
      double d = sensor_fusion[j][6];
     
      // check if the vehicle belong to lane that we are considering
      int traffic_lane  = getCurrentLane(d);
      if (traffic_lane == current_lane)
      {
        double vx = sensor_fusion[j][3];
        double vy = sensor_fusion[j][4];
        double car_speed = sqrt(vx*vx + vy*vy);
              
        double s = sensor_fusion[j][5];
        s += (double)previous_size*0.02*car_speed;
        
        // check if there is a car in that lane in +/- 60 meters from current car position
        double car_traffic_distance = s - car_s ;
        // Check if there is any car behind and we have safe space to perform Lane change
        if ((-10 < car_traffic_distance) &&  (car_traffic_distance < 30))
        {
          std::cout << "CANT GO TO LANE: " << current_lane << std::endl;
          best_lanes[i] = -1;
          break;
        }
      }
    }
  }
  return best_lanes;
}

// This function determines the traffic in the future lanes
// this will avoid making unnecessary lane change 
vector<int> getTrafficStateInLanes(const vector<int> &next_lanes, const vector<vector<double>> &sensor_fusion,
                                const double &car_s, const int &previous_size)
{
  int next_lane_size = next_lanes.size();
  vector<int> cars_in_lane(next_lane_size, -1);
  double MAX_DISTANCE = 200; //mts
  for(int i =0; i < next_lane_size; i++)
  {
    // This will check what is the traffic in the lane for 250 m ahead 
    double traffic_index = 0;
    int current_lane = next_lanes[i];
    
    // iterate over all potential future lanes
    for(int j = 0; j < sensor_fusion.size(); j++)
    {
      double d = sensor_fusion[j][6];
      int traffic_lane  = getCurrentLane(d);
      // check if the vehicle belong to lane that we are considering
      if (traffic_lane == current_lane)
      {
        double vx = sensor_fusion[j][3];
        double vy = sensor_fusion[j][4];
        double car_speed = sqrt(vx*vx + vy*vy);
              
        double s = sensor_fusion[j][5];
        s += (double)previous_size*0.02*car_speed;
        double distance = s - car_s;
        
        if (distance > -20  && distance < MAX_DISTANCE)
        {
          traffic_index += 1;
          
        }
      }
    }
  cars_in_lane[i] = traffic_index;
  }
  return cars_in_lane;
}

// Determine the ptsx and ptsy to fit the spline
void generateVectorToFit(
  							vector<double> &ptsx, vector<double> &ptsy, 
                            const double &car_s, const double &car_d, 
  							const double &car_x, const double &car_y, const double &car_yaw,
  							double &ref_x, double &ref_y, double &ref_yaw, 
  							const int &lane, const int &new_lane,
                            const vector<double> &maps_s, const vector<double> &maps_x,  const vector<double> &maps_y,
  							const vector<double> &previous_path_x, const vector<double> &previous_path_y
                           )
{
  int previous_size = previous_path_x.size();
  if (previous_size < 2)
  {
      double pref_x = car_x - cos(car_yaw);
      double pref_y = car_y - sin(car_yaw);
      
      ptsx.push_back(pref_x);
      ptsx.push_back(car_x);

      ptsy.push_back(pref_y);
      ptsy.push_back(car_y);
   }
   else
   {
      ref_x = previous_path_x[previous_size-1];
      ref_y = previous_path_y[previous_size-1];

      double prev_ref_x = previous_path_x[previous_size-2];
      double prev_ref_y = previous_path_y[previous_size-2];

      ref_yaw = atan2(ref_y - prev_ref_y ,ref_x - prev_ref_x);

      ptsx.push_back(prev_ref_x);
      ptsx.push_back(ref_x);

      ptsy.push_back(prev_ref_y);
      ptsy.push_back(ref_y);
  }
  // determine the last points for fit the curve based on the Behavior planner's lane output
	 vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * new_lane, maps_s, maps_x, maps_y);
     vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * new_lane, maps_s, maps_x, maps_y);
     vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * new_lane, maps_s, maps_x, maps_y);

     ptsx.push_back(next_wp0[0]);
     ptsx.push_back(next_wp1[0]);
     ptsx.push_back(next_wp2[0]);

     ptsy.push_back(next_wp0[1]);
     ptsy.push_back(next_wp1[1]);
     ptsy.push_back(next_wp2[1]);

}  
// This function converts from Global Coordinate system to Car Coordinate system
void convert_from_global_to_car_cordinate(
  										   const double &ref_x, const double &ref_y, const double &ref_yaw, 
  									 	   vector<double> &ptsx, vector<double> &ptsy
                                          )
{
  for(int i=0; i< ptsx.size(); i++)
  {
    double prev_x = ptsx[i];
    double prev_y = ptsy[i];
            
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
            
    ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));

    }
}

#endif  // HELPERS_H
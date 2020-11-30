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




// This function will provide the list of lanes where I can switch next
vector<int> getProspectiveLanesToChange(int &lane)
{
  vector<int> lanes;
  std::cout << "Current Lane of car: " << lane << std::endl;
  for(int i=-1; i < 2; i++)
  {
    int new_lane = lane + i;
    if ((0 <= lane <= 2) && (new_lane != lane))
    {
      lanes.push_back(new_lane);
      std::cout << "MIght now go to " << new_lane << std::endl;
    }
  }
  return lanes;
}

// Determine which is the best lane for this given scenario to take
vector<int> getBestLaneToChange(const vector<int> &next_lanes, const vector<vector<double>> &sensor_fusion,
                                const double &car_s, const int &previous_size)
{
  vector<int> best_lanes = {next_lanes.size(), 1};
  for(int i =0; i < next_lanes.size(); i++)
  {
    int current_lane = next_lanes[i];
    std::cout << "getBestLaneToChange: Evaluating Lane : " << current_lane << std::endl;
    // iterate over all potential future lanes
    for(int j = 0; j < sensor_fusion.size(); j++)
    {
      double d = sensor_fusion[i][6];
      // check if the vehicle belong to lane that we are considering
      if ((2 + current_lane*4 + 2 ) <= d <=(2 + current_lane*4 - 2))
      {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double car_speed = sqrt(vx*vx + vy*vy);
              
        double s = sensor_fusion[i][5];
        s += (double)previous_size*0.02*car_speed;
        // check if there is a car in that lane in +/- 60 meters from current car position
        if (car_s - 60 <= s <= car_s + 80)
        {
          std::cout << "getBestLaneToChange: There seems to be a car in the '" << current_lane << 
            "', Hence not a good option to change lane to " << current_lane <<std::endl;
          best_lanes[i] = -1;
          break;
        }
      }
    }
    for(int i = 0; i < best_lanes.size(); i++)
    {
      std::cout << "getBestLaneToChange: Lane values : " << best_lanes[i] << std::endl;
    }
  }
  return best_lanes;
}

// Determine the ptsx and ptsy to fit the spline
void generate_vector_to_fit(
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
  //     std::cout << "Initial pref_x value: " << pref_x << std::endl;
  // 	std::cout << "Initial pref_y value: " << pref_y << std::endl;
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
  int temp_value = new_lane - lane;
  
  for(int i=1; i<5; i++)
  {
 
    double new_car_s = car_s + i*20;
    double new_car_d = car_d + i*temp_value;
    
    vector<double> new_xy = getXY(new_car_s, new_car_d, maps_s, maps_x, maps_y);
    
    ptsx.push_back(new_xy[0]);
    ptsy.push_back(new_xy[1]);
  }
  
  std::cout << "Value Generated currently for PTSX and PTSY: \n\n" << std::endl;
  for(int i =0 ; i< ptsx.size(); i++)
  {
  	std::cout << "ptsx[" << i << "] = " <<  ptsx[i] << " ,";
    std::cout << "ptsy[" << i << "] = " <<  ptsy[i] << std::endl;
  }
  
  std::cout << std::endl << std::endl;
}  

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
            
//             std::cout << "ptsx[" << i << "]: Before: "<< prev_x << " After: " << ptsx[i] << std::endl;
//             std::cout << "ptsy[" << i << "]: Before: "<< prev_y << " After: " << ptsy[i] << std::endl;
        
    }
}

#endif  // HELPERS_H
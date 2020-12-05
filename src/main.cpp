#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  double MAX_SPEED = 49.5;
  double MAX_ACCELERATION = 0.224;
  double ref_speed = 0;// MPH
  double MAX_DISTANCE_FROM_FRONT_CAR = 30; //m
  
  h.onMessage([&ref_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &MAX_SPEED, &MAX_ACCELERATION,
               &MAX_DISTANCE_FROM_FRONT_CAR]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
	  
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
		  
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
		  
          // OUR CODE
          int previous_size = previous_path_x.size();
          bool too_close = false;  
          bool decrease_speed = false;
          int lane = getCurrentLane(car_d);
          int next_lane = lane;
          
          // Compute if car is ahead of us
          if (previous_size > 0)
          {
            car_s = end_path_s;
          }
          
          // Check if there is any car ahead
          for(int i=0; i< sensor_fusion.size(); i++)
          {
            double d = sensor_fusion[i][6];
            // check if the car is our lane
            if (d <= (2+4*lane+2) && d >= (2 + 4*lane - 2))
            {
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double car_speed = sqrt(vx*vx + vy*vy);
              
              double s = sensor_fusion[i][5];
              s += (double)previous_size*0.02*car_speed;
              
              if ((s > car_s) && (s - car_s < MAX_DISTANCE_FROM_FRONT_CAR))
              {
                std::cout << "Car too close !!!!!" << std::endl;
                too_close = true;
              }
              
            }
          }
          
          //Check if there is car ahead
          if(too_close)
          {
            //Getthe list of lanes that we can consider to perform lane change
            vector<int> next_lanes = getProspectiveLanesToChange(lane);
            // Out of list of Lanes we can perform Lane Change evaluate which lanes are safe to perform Lane change
            vector<int> change_lane = getBestLaneToChange(next_lanes, sensor_fusion, car_s, previous_size);
            // Check what is the density of Traffic[Number of cars ahead in the Future Lane] incase we have 2 Lanes to choose from. In case of when the car is in lane 1 and lane 2 and lne 0 are safe to perform Lane change
            vector<int> traffic_index = getTrafficStateInLanes(next_lanes, sensor_fusion, car_s, previous_size);
            int MAX_CARS = 99999;
            // Select the Lane that is safe to perform Lane Chnage and Has less traffic ahead in that lane
            for(int i = 0; i< change_lane.size(); i++)
            {
              int temp_value = change_lane[i];
              // Check if its safe to Consider the Lane
              if (temp_value != -1 && traffic_index[i] < MAX_CARS)
              {
                next_lane = next_lanes[i];
                std::cout << "selected Lane: " << next_lane  << " Traffic: " << traffic_index[i] << std::endl;
                MAX_CARS = traffic_index[i];
                decrease_speed = true;
    			//break; // IF need to take right lane as priority
              }
            }
            
            // If none of the lanes are safe to change then decrease speed
            if (next_lane == lane)
            {
              std::cout << " CANT CHANGE LANE" << std::endl;
              decrease_speed = true;
            }
            else
            {
              std::cout << "Lane change from Lane: " << lane << " to  Lane: " << next_lane << std::endl;
            }
          }


          // Change Speed and  make sure that car is not going back
          if((decrease_speed) && (ref_speed > 0))
          {
            std::cout << "Decreasing speed to " << ref_speed << std::endl;
            ref_speed -= MAX_ACCELERATION;
          }
          else if(ref_speed < MAX_SPEED)
          {
            ref_speed += MAX_ACCELERATION;
            std::cout << "Increasing speed to " << ref_speed << std::endl;
          }
            
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          vector<double> ptsx;
          vector<double> ptsy;
          //Generate the set of values to fit the spline function
          generateVectorToFit(
            					 ptsx, ptsy, 
                                 car_s, car_d, car_x, car_y, car_yaw,
            					 ref_x, ref_y, ref_yaw,
                                 lane, next_lane,
                                 map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                 previous_path_x, previous_path_y
                                );
          
          // Convert the present XY Global coordinate to Local Car's coordinates
          convert_from_global_to_car_cordinate(
            									ref_x, ref_y, ref_yaw,
            									ptsx, ptsy
          									  ); 
          // Fit the points with spline
          tk::spline s;
   		  s.set_points(ptsx,ptsy);
          
          // Determine the spacing between points for optimal Jerk Free path
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Get the last XY values let over from previour_path
          for(int i=0; i< previous_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Set maximum number of entries in the next_x_vlas and next_y_vals
          int MAX_VALUES_IN_NEXT = 12;
          double x_add_on = 0;
		  
          // Generate new values for trajectory
          for(int i=1; i <= MAX_VALUES_IN_NEXT -previous_size; i++)
          {
            double temp = (0.02 * ref_speed) / 2.24;
            double N = (target_dist / temp);
            
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            double before_x = x_point;
            double before_y = y_point;
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
		  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
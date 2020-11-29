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
  
  int lane = 1;
  double MAX_SPEED = 49.5;
  double MAX_ACCELERATION = 0.224;
  double ref_speed = 0;// MPH
  
  h.onMessage([&ref_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &MAX_SPEED, &MAX_ACCELERATION]
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
		  
          int previous_size = previous_path_x.size();
          bool too_close = false;
          // Compute if car is ahead of us
          if (previous_size > 0)
          {
            car_s = end_path_s;
          }
          
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
              
              if ((s > car_s) && (s - car_s < 30))
              {
                std::cout << "Car too close !!!!!" << std::endl;
                too_close = true;
              }
              
            }
          }
          
          if(too_close)
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
          
//           std::cout << "Initial X value: " << car_x << std::endl;
//           std::cout << "Initial Y value: " << car_y << std::endl;
          vector<double> ptsx;
          vector<double> ptsy;
          
          if (previous_size < 2)
          {
            double pref_x = car_x - cos(car_yaw);
            double pref_y = car_y - sin(car_yaw);
//             std::cout << "Initial pref_x value: " << pref_x << std::endl;
//             std::cout << "Initial pref_y value: " << pref_y << std::endl;
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
          
          // Calculate 3 POints ahead to fit the Spline function
          vector<double> wp0 = getXY(car_s+30, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1 = getXY(car_s+60, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s+90, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
//           std::cout << "WP0 x value: " << wp0[0] << ", WP0 y value: "<< wp0[1] <<std::endl;
//           std::cout << "WP1 x value: " << wp1[0] << ", WP1 y value: "<< wp1[1] <<std::endl;
//           std::cout << "WP2 x value: " << wp2[0] << ", WP2 y value: "<< wp2[1] <<std::endl;
          
          ptsx.push_back(wp0[0]);
          ptsx.push_back(wp1[0]);
          ptsx.push_back(wp2[0]);
          
          ptsy.push_back(wp0[1]);
          ptsy.push_back(wp1[1]);
          ptsy.push_back(wp2[1]);
          
          // Convert the present XY Global coordinate to Local Car's coordinates
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
          
          // Fit the 5 points with spline
          tk::spline s;
   		  s.set_points(ptsx,ptsy);
          
          // Determine the spacing between points for optimal Jerk Free path
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          
          double x_add_on = 0;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Get the last XY values let over from previour_path
          for(int i=0; i< previous_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Add the new points to list by converting them back to Global coordinate system
//           std::cout << "New updated x y values";
          for(int i=1; i <= 50-previous_size; i++)
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
            
//             std::cout << "next_x_vals[" << i << "]: Before: "<< before_x << " After: " << x_point << std::endl;
//             std::cout << "next_y_vals[" << i << "]: Before: "<< before_y << " After: " << y_point << std::endl;
            
          }
//           for(int i =0; i < next_x_vals.size(); i++)
//           {
//             std::cout << "next_x_vals[ "<< i <<"]: " << next_x_vals[i] << "   " ;
//             std::cout << "next_y_vals[ "<< i <<"]: " << next_y_vals[i] << std::endl;
//           }
//           std::cout << std::endl << std::endl << std::endl;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  
          
		  
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
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

  // My car starts in middle lane (1)
  int lane = 1;

  // Reference velocity of my car to target
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          // Localization >>
          // My car's localisation data provided by simulator
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous Path >>
          // Previous path data sent to the planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Previous path size
          int prev_size = previous_path_x.size();

          // Sensor Fusion >>
          // Provides a list of all cars on the same side of the road
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          // QA video 40m00s
          if(prev_size > 0){
            car_s = end_path_s;
          }

          // Other velocity parameters for my car
          double speed_diff = 0;
          const double MAX_SPEED = 49.5;
          const double MAX_ACC = .224;

          // Lane position of other cars
          int check_car_lane;
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          bool too_close = false;

          // Load data about other cars from sensors
          for(int i = 0; i < sensor_fusion.size(); i++){

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            // Determine lane of other cars by their d distance from centre of road
            float d = sensor_fusion[i][6];

            if(d > 8){
              // right lane
              check_car_lane = 2;
            }
            else if(d > 4){
              // middle lane
              check_car_lane = 1;
            }
            else{
              // left lane
              check_car_lane = 0;
            }

            // If using previous points then project s value outputs
            check_car_s += ((double)prev_size * 0.02 * check_speed);

            // Check other car lane positions relative to my car and if their s value is +/- 30m set boolean to true
            // Car in my lane
            if ( check_car_lane == lane ) {
              car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;

            // Car to the left
            } else if ( check_car_lane - lane == -1 ) {
              car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;

            // Car to the right
            } else if ( check_car_lane - lane == 1 ) {
              car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
          }

          // Path Planning >>
          // Decide lane or adjust speed
          // Car ahead
          if ( car_ahead ) {
            // If there is no car left and there is a left lane
            if ( !car_left && lane > 0 ) {
              // Change lane left
              lane--;
            // If there is no car right and there is a right lane
            } else if ( !car_right && lane != 2 ){
              // Change lane right
              lane++;
            // If unable to change lane
            } else {
              // Reduce speed
              speed_diff -= MAX_ACC;
            }
          }
          // Road ahead is clear
          else {
            // If my car is not in the middle lane
            if ( lane != 1 ) {
              // If my car is left lane and no cars to the right
              // OR if my car is right lane and no cars to the left
              if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) {
                // Change lane back to the middle lane
                lane = 1;
              }
            }
            // If my speed is less than the speed limit
            if ( ref_vel < MAX_SPEED ) {
              // Accelerate at maximum
              speed_diff += MAX_ACC;
            }
          }

          // Generating Paths >>
          // Create a list of widely spaced (x,y) waypoints that are evenly spaced at 30m intervals
          // Use splines for my car to follow smooth path through all waypoints - QA video from 21m30s
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous path size is almost empty, use the car as starting reference
          if(prev_size < 2){

            //Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          // Use the previous path's end point as starting reference
          else{

            // Redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's end points
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {

            // Shift and rotate car reference angle to 0 degrees in Frenet
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

          }

          // Spline >>
          tk::spline s;

          // Set (x,y) points for the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all the previous path points from last time
          for(int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up the spline points so that we travel at our reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

          double x_add_on = 0;

          // Fill up the rest of the path planner after filling it with previous set_points
          for (int i = 1; i <= 50 - prev_size; i++){

            // Set velocity of my car
            ref_vel += speed_diff;
            if ( ref_vel > MAX_SPEED ) {
              ref_vel = MAX_SPEED;
            } else if ( ref_vel < MAX_ACC ) {
              ref_vel = MAX_ACC;
            }

            // Set (x,y) points of the spline
            double N = (target_dist / (0.02 * ref_vel / 2.24));  // mph to m/s
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          // Json Message >>
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          // Web Sockets >>
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

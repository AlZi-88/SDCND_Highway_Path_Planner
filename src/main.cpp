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

#define FOLLOW_LANE 0
#define PLCL 1
#define PLCR 2
#define LCL 3
#define LCR 4

struct car{
  double x;
  double y;
  double s;
  double predicted_s;
  double d;
  double yaw;
  double speed;
};

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

  double lane = 1.0;
  double speed_tar = 49.5;
  double speed_sp = 0.0;
  int state_vehicle = FOLLOW_LANE;
  double time_lane_change = 5;

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





  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &speed_sp, &speed_tar, &state_vehicle, &time_lane_change]
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





          int prev_size = previous_path_x.size();

          if (prev_size > 0){
            car_s = end_path_s;
          }

          vector < car > front_cars;
          vector < car > left_cars;
          vector < car > right_cars;

          for (int i = 0; i< sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            if (d< (2+4*lane+2) && d > (2+4*lane -2)){
              //then car is in my lane
              car front_car;
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              front_car.d = d;
              front_car.speed = sqrt(vx*vx + vy*vy);
              front_car.s = sensor_fusion[i][5];

              front_car.predicted_s = front_car.s + (double)prev_size*.02*front_car.speed;
              if (front_car.predicted_s > car_s){
                //car is really in fron of me
                front_cars.push_back(front_car);
              }

            }else if(d< (2+4*(lane-1)+2) && d > (2+4*(lane-1) -2)){
              //then car is on left lane
              car left_car;
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              left_car.d = d;
              left_car.speed = sqrt(vx*vx + vy*vy);
              left_car.s = sensor_fusion[i][5];

              left_car.predicted_s = left_car.s + (double)prev_size*.02*left_car.speed;
              left_cars.push_back(left_car);

            }else if(d< (2+4*(lane+1)+2) && d > (2+4*(lane+1) -2)){
              //then car is on left lane
              car right_car;
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              right_car.d = d;
              right_car.speed = sqrt(vx*vx + vy*vy);
              right_car.s = sensor_fusion[i][5];

              right_car.predicted_s = right_car.s + (double)prev_size*.02*right_car.speed;
              right_cars.push_back(right_car);


            }
          }
          std::cout << "number of front vehicles "<< front_cars.size() << std::endl;
          std::cout << "number of left vehicles "<< left_cars.size() << std::endl;
          std::cout << "number of right vehicles "<< right_cars.size() << std::endl;
          int closest=0;
          double distance=9999999.0;
          bool front_vehicle_close = false;
          for (int i = 0; i< front_cars.size(); i++){

            if ((distance > (front_cars[i].predicted_s - car_s)) && ((front_cars[i].predicted_s - car_s) < 30)){
              //then other car is coming close, so reduce velocity
              //speed_tar = 29.5;

              closest = i;
              distance = front_cars[i].predicted_s - car_s;
              front_vehicle_close = true;
            }

          }

          double s_min = car_s - 10;
          double s_max = car_s +30;
          bool left_lane_blocked = false;
          for (int i = 0; i< left_cars.size(); i++){

            if (left_cars[i].predicted_s > s_min && left_cars[i].predicted_s <= s_max){
              //left lane is occupied by another car
              //speed_tar = 29.5;

              left_lane_blocked = true;
            }

          }

          bool right_lane_blocked = false;
          for (int i = 0; i< right_cars.size(); i++){

            if (right_cars[i].predicted_s > s_min && right_cars[i].predicted_s <= s_max){
              //left lane is occupied by another car
              //speed_tar = 29.5;

              right_lane_blocked = true;
            }

          }
          std::cout << "Front vehicle is close " << front_vehicle_close << std::endl;
          std::cout << "Left lane is blocked " << left_lane_blocked << std::endl;
          std::cout << "Right lane is blocked " << right_lane_blocked << std::endl;

          if (state_vehicle == FOLLOW_LANE){
            speed_tar = 49.5;
            if (front_vehicle_close){
              speed_tar = front_cars[closest].speed*2.24;
              if (lane > 0 && time_lane_change <= 0){
                state_vehicle = PLCL;
              }else if (lane < 2 && !right_lane_blocked&& time_lane_change <= 0){
                state_vehicle = PLCR;
              }
            }else if (lane < 2 && !right_lane_blocked && time_lane_change <= 0){
              state_vehicle = PLCR;
            }
            if (time_lane_change >= 0){
              time_lane_change -= 0.02;
            }
          }else if(state_vehicle == PLCL){
            if (front_vehicle_close){
              speed_tar = front_cars[closest].speed*2.24;
            }
            if (!left_lane_blocked){
              state_vehicle = LCL;
            }else if (distance > 30){
              state_vehicle = FOLLOW_LANE;
            }
          }else if(state_vehicle == PLCR){

            if (front_vehicle_close){
              speed_tar = front_cars[closest].speed*2.24;
            }
            if (!right_lane_blocked){
              state_vehicle = LCR;
            }else if (distance > 30){
              state_vehicle = FOLLOW_LANE;
            }
          }else if(state_vehicle == LCL){
            lane -= 1;
            state_vehicle = FOLLOW_LANE;
            time_lane_change = 5;
          }else if(state_vehicle == LCR){
            lane += 1;
            state_vehicle = FOLLOW_LANE;
            time_lane_change = 5;
          }

          if(speed_sp < speed_tar){
            //accelerate
            speed_sp += .224;
          } else if (speed_sp > speed_tar){
            speed_sp -= .224;
          }
          std::cout << "vehicle state = "<< state_vehicle << "timer = " << time_lane_change << std::endl;
          if (front_cars.size() > 0){
            std::cout << "front vehicle speed of car number "<< closest << " is " << front_cars[closest].speed*2.24 << std::endl;
          }

          vector<double> points_x;
          vector<double> points_y;


          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size<2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            points_x.push_back(prev_car_x);
            points_x.push_back(car_x);

            points_y.push_back(prev_car_y);
            points_y.push_back(car_y);
          }else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            points_x.push_back(prev_ref_x);
            points_x.push_back(ref_x);

            points_y.push_back(prev_ref_y);
            points_y.push_back(ref_y);
          }

          vector <double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          points_x.push_back(next_wp0[0]);
          points_x.push_back(next_wp1[0]);
          points_x.push_back(next_wp2[0]);

          points_y.push_back(next_wp0[1]);
          points_y.push_back(next_wp1[1]);
          points_y.push_back(next_wp2[1]);

          for (int i =0; i<points_x.size(); i++){
            //std::cout << "points_x_before_trans[" << i << "] = " << points_x[i] << std::endl;
            //std::cout << "points_y_before_trans[" << i << "] = " << points_y[i] << std::endl;
            double shift_x = points_x[i] - ref_x;
            double shift_y = points_y[i] - ref_y;

            points_x[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            points_y[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            //std::cout << "points_x[" << i << "] = " << points_x[i] << std::endl;
            //std::cout << "points_y[" << i << "] = " << points_y[i] << std::endl;
          }

          tk::spline spl;
          spl.set_points(points_x, points_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;


          for (int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double x_tar = 30.0;
          double y_tar = spl(x_tar);
          double dist_tar = sqrt((x_tar*x_tar) + (y_tar*y_tar));

          double x_add_on = 0;

          for (int i = 1; i <= 50-previous_path_x.size(); i++){

            double N = (dist_tar/(.02*speed_sp/2.24));
            double x_point = x_add_on + x_tar/N;
            double y_point = spl(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            //backward rotation for map coordinates

            x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

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

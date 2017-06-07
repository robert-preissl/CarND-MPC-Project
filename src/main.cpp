#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"


// for plotting CTE errors
//#define _SAVE_CTE_DATA_

#ifdef _SAVE_CTE_DATA_
  std::string file_name_cte = "/Users/robert/SDCND/Term2/CarND-MPC-Project/data/mpc_cte_3.txt";
  std::ofstream datafile_cte (file_name_cte);

  std::string file_name_acc = "/Users/robert/SDCND/Term2/CarND-MPC-Project/data/mpc_acc.txt";
  std::ofstream datafile_acc (file_name_acc);

  int t = 0;
#endif


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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// 2D coordinate transformation (first Translation then Rotation)
//  from map coordinates to vehicle coordinates
void CoordTransf2D(const double & x_in,
                   const double & y_in,
                   const double & origin_x,
                   const double & origin_y,
                   double & x_out,
                   double & y_out,
                   double psi) {

  // 1. Translation
  double x_t = x_in - origin_x; // - 32 - -40 = 8   or -43 - - 40 = -3
  double y_t = y_in - origin_y; // 113 - 108 = 5    or 105 - 108 = -3

  // 2. Rotation
  // [X'] = [ cos sin] * [X]
  // [Y'] = [-sin cos] * [Y]
  x_out = x_t * cos(psi) + y_t * sin(psi); // -9  or 3.9
  y_out = - x_t * sin(psi) + y_t * cos(psi); // 0.8 or 0.7

}

// compute the car's reference path in Vehicle coordinates
void get_reference_path(const double & px,
                        const double & py,
                        const double & psi,
                        const vector<double> & ptsx,
                        const vector<double> & ptsy,
                        vector<double>& next_x_vals,
                        vector<double>& next_y_vals,
                        double v_metersPerSec,
                        double latency_milliSec) {

/*
the x axis always points in the direction of the car’s heading and the y axis
 points to the left of the car.
 So if you wanted to display a point 10 units directly in front of the car,
 you could set next_x = {10.0} and next_y = {0.0}.

Remember that the server returns waypoints using the map's coordinate system,
which is different than the car's coordinate system
*/

/*
                                        map-x  <--------------------------
                           x-axis                                        |
                     pts                                                 |
                     x     ||  pts=[(-32,113), (-43,105), (-61,92),      |
  y-axis             x     ||      (-78,78), (-93,65)]  ... ref. path    |
    ====              x    ||                                            |
        ====           x  ||                                             |
            ====       x ||                                              |
                ==== Vehicle (px=-40,py=108,psi=5)                       |
                                                                         |
                                                                         v  map-y
*/

  double latency_sec = latency_milliSec / 1000.0;

  // convert to vehicle space
  for (int i = 0; i < ptsx.size(); i++) {
    double x, y;
    CoordTransf2D(ptsx[i], ptsy[i], px, py, x, y, psi);

    // after the coordinate transformation, we move the x coordinate
    //  (in vehicle space) by a certain distance in vehicle direction
    //  to account for latency of sending actuator commands to the simulator

    //x += v_metersPerSec * latency_sec;
    // TODO
    printf("\n XX1 -- x = %f / offset = %f \n", x, v_metersPerSec * latency_sec);

    next_x_vals.push_back( x ); // -9  or 3.9
    next_y_vals.push_back( y ); // 0.8 or 0.7
  }
}

double convert_milesPerHour_to_MetersPerSec(double milesPH) {
  return milesPH * 0.44704;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // map space
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"]; // map space
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v_milesPerHour = j[1]["speed"];
          double v_metersPerSec = convert_milesPerHour_to_MetersPerSec(v_milesPerHour);
          double latency_milliSec = 100;

          /*
          * Calculate steeering angle and throttle using MPC.
          */
          double steer_value;
          double throttle_value;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // get the ref. path in vehicle space
          // TODO: v stuff
          get_reference_path(px, py, psi, ptsx, ptsy, next_x_vals, next_y_vals, v_metersPerSec, latency_milliSec);

          // TODO: Implement latency compensation of the controller by predicting the state 100ms into the future before passing it to the solver.

          auto coeffs = polyfit(convert_vec_to_eigen<double>(next_x_vals), convert_vec_to_eigen<double>(next_y_vals), 3); // TODO

          double latency_sec = latency_milliSec / 1000.0;
          double x_lat = v_metersPerSec * latency_sec;
          double cte = polyeval(coeffs, x_lat); // TODO // evaluation done in vehicle space -> hence, we compute CTE to y=0

#ifdef _SAVE_CTE_DATA_
          if (!datafile_cte.is_open())
          {
            datafile_cte.open(file_name_cte, std::ios_base::app);
          }
          datafile_cte << t << " " << cte << "\n";
          if (datafile_cte.is_open())
            datafile_cte.close();
#endif

          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x + coeffs[2] * x ^ 2 + coeffs[3] * x ^ 3   -> coeffs[1]
          double epsi = -atan(coeffs[1] + 2 * x_lat * coeffs[2] + 3 * x_lat * x_lat * coeffs[3]);

          Eigen::VectorXd state(6);

// Recall the equations for the model:
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

// TODO: explain

          double old_steer_value = j[1]["steering_angle"];
          double old_throttle_value = j[1]["throttle"];

          // y_t = 0 since y_lat = v[t] * sin(psi[t]) * dt   and sin = 0

      /*    double y_lat = 0;
          double psi_lat = 0;
          double v_lat = v_metersPerSec;
          double cte_lat = cte;
          double epsi_lat = epsi;*/

          double y_lat = 0;
          double psi_lat = 0;// TODO: steer is 0 - v_metersPerSec * old_steer_value * latency_milliSec / Lf;
          double v_lat = 0;// v_metersPerSec + old_throttle_value * latency_milliSec;
          double cte_lat = 0;//cte + v_metersPerSec * sin(epsi) * latency_milliSec;
          double epsi_lat = epsi; // - v_metersPerSec * old_steer_value * latency_milliSec / Lf;

          // state << x_lat, y_lat, psi_lat, v_lat, cte_lat, epsi_lat; // in Vehicle coordinates, the first three states are 0 (x,y and steering)
          state << x_lat, 0, 0, v_milesPerHour, cte, epsi;

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          vector<double> mpc_psi_vals;
          vector<double> mpc_v_vals;
          vector<double> mpc_cte_vals;
          vector<double> mpc_epsi_vals;
          vector<double> mpc_delta_vals;
          vector<double> mpc_a_vals;

          mpc.Solve(state,
                    coeffs,
                    mpc_x_vals,
                    mpc_y_vals,
                    mpc_psi_vals,
                    mpc_v_vals,
                    mpc_cte_vals,
                    mpc_epsi_vals,
                    mpc_delta_vals,
                    mpc_a_vals
          );

#ifdef _SAVE_CTE_DATA_
          if (!datafile_acc.is_open())
          {
            datafile_acc.open(file_name_acc, std::ios_base::app);
          }
          datafile_acc << t << " " << mpc_delta_vals[0] << " " << mpc_a_vals[0] << "\n";
          if (datafile_acc.is_open())
            datafile_acc.close();

          t++;
#endif

          throttle_value = mpc_a_vals[0];
          steer_value    = mpc_delta_vals[0];

          json msgJson;
          msgJson["steering_angle"] = -steer_value / STEER_LIMIT; // to ensure steer values are in between [-1, 1].
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory (green line) in vehicle space
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line (yellow line) in vehicle space
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds((int)latency_milliSec));
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

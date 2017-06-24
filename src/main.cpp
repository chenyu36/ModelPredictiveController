#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  static double acceleration = 0;
  static long long t_sleep_time_ms = 100;
  static double delta = 0;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (mpc.is_debug_active) {
      cout << sdata << endl;
    }
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * Declare the control input to the simulator
          * steeering angle and throttle
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          // Transform the map coordinate to car coordinate
          for (int i = 0; i < ptsx.size(); i++) {
            double relativeX = ptsx[i] - px;
            double relativeY = ptsy[i] - py;
            double psi_unity = psi - M_PI / 2;
            double rotatedX = cos(-psi_unity) * relativeX - sin(-psi_unity) * relativeY;
            double rotatedY = cos(-psi_unity) * relativeY + sin(-psi_unity) * relativeX;
            ptsx[i] = rotatedY;
            ptsy[i] = -rotatedX;
          }

          // Before fitting the polynomial, map the vector type to Eigen::VectorXd
          // because polyfit() function accepts the first 2 parameter type as Eigen::VectorXd
          Eigen::VectorXd coeffs;
          Eigen::VectorXd ptsx_e = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_e = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          // debug
          if (mpc.is_debug_active) {
            cout << "before" << endl;
            for (int i = 0; i < ptsx.size(); i++) {
              cout << "ptsx " << i << " " << ptsx[i] << "\t" << "ptsy " << i << " " << ptsy[i] << endl;
            }
            cout << "after" << endl;
            for (int i = 0; i < ptsx_e.size(); i++) {
              cout << "ptsx_e " << i << " " << ptsx_e[i] << "\t" << "ptsy_e " << i << " " << ptsy_e[i] << endl;
            }
          }


          if ((ptsx.size() > 0) && (ptsy.size() > 0) && (ptsx.size() == ptsy.size())) {
            // fit the polynomial with 3rd order curve
            coeffs = polyfit(ptsx_e, ptsy_e, 3);
          }

          // account for latency: time used by thread sleep + time used for Solve() function
          // during this time, the car continues to travel before the actuator command got sent back to the simulator
          double total_latency = t_sleep_time_ms/1000.0 + mpc.solve_time_;

          // debug
          if (mpc.is_debug_active) {
            cout << "mpc.average_solve_time_ " << mpc.solve_time_ << endl;
            cout << "total latency " << total_latency << endl;
          }
          const double convert_to_metric = 1600.0/3600; // convert from mph to m/s
          double dv = acceleration * total_latency;
          double dx = (v + dv) * convert_to_metric * total_latency;
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          // in car coordinate, (x, y) = (0, 0), but we account for dx caused by latency.
          double cte = polyeval(coeffs, dx) - 0;
          // epsi = psi - desired_psi
          // where psi is 0 in the car coordinate
          // desired_psi = arctan(f'(x)) where f(x) = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 + coeffs[3] * x^3
          // f'(x) = coeffs[1] + 2 * coeffs[2] * dx + 3 * coeffs[3] * dx * dx
          // again, in car coordinate, (x, y) = (0, 0), but we account for dx caused by latency.
          double d_psi = (v+dv) * convert_to_metric * delta * total_latency;
          double epsi = d_psi - atan(coeffs[1] + 2 * coeffs[2] * dx + 3 * coeffs[3] * dx * dx);

          // debug
          if (mpc.is_debug_active) {
            cout << "psi " << psi << endl;
            cout << "atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px) " << epsi << endl;
          }

          Eigen::VectorXd state(6);
          // in car coordinate, (x, y) = (0, 0), but we account for dx and dv caused by latency
          state << dx, 0, 0, (v + dv), cte, epsi;

          // Calculate steeering angle and throttle using MPC
          auto actuators = mpc.Solve(state, coeffs);

          // simulator's steering angle has the opposite sign of the actuator "delta"
          steer_value = -actuators[0];
          throttle_value = actuators[1];
          acceleration = actuators[1];
          delta = actuators[0];

          // debug
          if (mpc.is_debug_active) {
            cout << "steering_value " << steer_value << endl;
            cout << "throttle_value " << throttle_value << endl;
          }

          // restrict the range of the control input to the simulator
//          if (steer_value > 1) {
//            steer_value = 1;
//          }
//          if (steer_value < -1) {
//            steer_value = -1;
//          }
          if (throttle_value > 1) {
            throttle_value = 1;
          }
          if (throttle_value < -1) {
            throttle_value = -1;
          }

          // set the json object with the calculated control input to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Declare variables for displaying the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          int mpc_val_size = mpc.mpc_x_vals_.size();
          mpc_x_vals.resize(mpc_val_size);
          mpc_y_vals.resize(mpc_val_size);
          for (int i = 0; i < mpc_val_size; i++) {
            mpc_x_vals[i] = mpc.mpc_x_vals_[i];
            mpc_y_vals[i] = mpc.mpc_y_vals_[i];
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          next_x_vals.resize(ptsx.size());
          next_y_vals.resize(ptsy.size());

          next_x_vals = ptsx;
          next_y_vals = ptsy;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          auto t_before_sleep = chrono::high_resolution_clock::now();
          if (mpc.is_debug_active) {
            std::cout << msg << std::endl;
          }
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));

          // Measure actual sleep time
          auto t_sleep_time = chrono::high_resolution_clock::now() - t_before_sleep;
          t_sleep_time_ms = chrono::duration_cast<chrono::milliseconds>(t_sleep_time).count();

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

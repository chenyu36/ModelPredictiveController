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
double normalizeAngle(double psi) {
  const double Max = M_PI;
  const double Min = -M_PI;

  return psi < Min
         ? Max + std::fmod(psi - Min, Max - Min)
         : std::fmod(psi - Min, Max - Min) + Min;
}

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



  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          // transform the map coordinate to car coordinate
          for (int i = 0; i < ptsx.size(); i++) {
            double relativeX = ptsx[i] - px;
            double relativeY = ptsy[i] - py;
            double psi_unity = psi - M_PI/2;
            double rotatedX = cos(-psi_unity) * relativeX - sin(-psi_unity) * relativeY;
            double rotatedY = cos(-psi_unity) * relativeY + sin(-psi_unity) * relativeX;
            ptsx[i] = rotatedY;
            ptsy[i] = -rotatedX;
          }


          // Before fitting the polynomial, map the vector type to Eigen::VectorXd
          // because polyfit() function accept the first 2 parameter type as Eigen::VectorXd
          Eigen::VectorXd coeffs;
          Eigen::VectorXd ptsx_e = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_e = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());



          // debug
//          cout << "before" << endl;
//          for (int i=0; i < ptsx.size(); i++) {
//            cout << "ptsx " << i << " " << ptsx[i] << "\t" << "ptsy " << i << " " << ptsy[i] << endl;
//          }
//          cout << "after" << endl;
//          for (int i=0; i < ptsx_e.size(); i++) {
//            cout << "ptsx_e " << i << " " << ptsx_e[i] << "\t" << "ptsy_e " << i << " " << ptsy_e[i] << endl;
//          }


          if ((ptsx.size() > 0) && (ptsy.size() > 0) && (ptsx.size() == ptsy.size())) {
            // fit the polynomial with 2nd order curve
            coeffs = polyfit(ptsx_e, ptsy_e, 3);
          }

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, 0) - 0;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 -> coeffs[1] + 2 * coeffs[2] * x
          double epsi = 0 - atan(coeffs[1] + 2 * coeffs[2] * 0 + 3 * coeffs[3] * 0 * 0);

          // debug
          cout << "psi " << psi << endl;
          cout << "atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px) " << epsi << endl;

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;  // TODO: test to see if psi can be used as is or in the form of (psi - M_PI/2)

          auto vars = mpc.Solve(state, coeffs);
          steer_value = -vars[2];
          throttle_value = vars[3];
          // debug
          cout << "steering_value " << steer_value << endl;
          cout << "throttle_value " << throttle_value << endl;

          if (steer_value > 1) {
            steer_value = 1;
          }
          if (steer_value < -1) {
            steer_value = -1;
          }

          if (throttle_value > 1) {
            throttle_value = 1;
          }
          if (throttle_value < -1) {
            throttle_value = -1;
          }
//          steer_value = -1.0*M_PI/180.0; // TODO: test only, remove after test. Curve to the left
//          throttle_value = 0.05;// TODO: test only, remove after test
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
//          mpc_x_vals = {0, 5, 10, 15, 20, 25};
//          mpc_y_vals = {0, 0, 0, 0, 0, 0};
          double mpc_predicted_x = vars[0];
          double mpc_predicted_y = vars[1];
          double relativeX = mpc_predicted_x - px;
          double relativeY = mpc_predicted_y - py;
          double psi_unity = psi - M_PI/2;
          double rotatedX = cos(-psi_unity) * relativeX - sin(-psi_unity) * relativeY;
          double rotatedY = cos(-psi_unity) * relativeY + sin(-psi_unity) * relativeX;
          cout << "predicted x value " << rotatedY << endl;
          cout << "predicted y value " << -rotatedX << endl;
          mpc_x_vals.push_back(rotatedY);
          mpc_y_vals.push_back(-rotatedX);


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

//          next_x_vals = {0, 10, 20, 30, 40, 50};
//          next_y_vals = {0, 0, 0, 0, 0, 0};
          // transform the waypoints from map coordinate to vehicle coornidate
//          relativeX = B.x - A.x
//          relativeY = B.y - A.y
//          rotatedX = Cos(-Angle) * relativeX - Sin(-Angle) * relativeY
//          rotatedY = Cos(-Angle) * relativeY + Sin(-Angle) * relativeX
//          next_y_vals = {0, 0, 0, 0, 0, 0};
          next_x_vals.resize(ptsx.size());
          next_y_vals.resize(ptsy.size());

//          if (ptsx.size() > 0 && (ptsx.size() == ptsy.size())) {
//            for (int i = 0; i < ptsx.size(); i++) {
//              double relativeX = ptsx[i] - px;
//              double relativeY = ptsy[i] - py;
//              double psi_unity = psi - M_PI/2;
//              double rotatedX = cos(-psi_unity) * relativeX - sin(-psi_unity) * relativeY;
//              double rotatedY = cos(-psi_unity) * relativeY + sin(-psi_unity) * relativeX;
//              next_x_vals[i] = rotatedY;
//              next_y_vals[i] = -rotatedX;
//            }
//          }

          next_x_vals = ptsx;
          next_y_vals = ptsy;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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

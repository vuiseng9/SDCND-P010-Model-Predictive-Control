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

extern size_t N;

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
    cout << sdata << endl;
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
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Transform waypts from map reference to car reference
          // Translation and Rotation
          Eigen::VectorXd wayptsx(ptsx.size());
          Eigen::VectorXd wayptsy(ptsy.size());

          for (int i = 0; i < ptsx.size(); i++) { 
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            wayptsx(i) = (dx * cos(-psi) - dy * sin(-psi));
            wayptsy(i) = (dx * sin(-psi) + dy * cos(-psi));
          }

          // Fit waypts to a 3rd order polynomial
          auto C = polyfit(wayptsx, wayptsy, 3);

          // Initial State
          // Since the State takes car as reference system 
          // some of the state will be zero as initial state
          double x_     = 0;
          double y_     = 0;
          double psi_   = 0;
          double v_     = v;
          double cte_   = polyeval(C, x_) - y_; 
          // most term of epsi_ will be zero since x_ is zero
          double epsi_  = psi_ - atan(3*C[3]*pow(x_,2) + 2*C[2]*x_ + C[1]); 

          // Account actuator dynamic delay by taking state 100ms in advance
          double latency_sec = 0.1;
          double Lf = 2.67;
          double x_delay    = x_    + v_ * cos(psi_) * latency_sec;
          double y_delay    = y_    + v_ * sin(psi_) * latency_sec;
          double psi_delay  = psi_  - v_ / Lf * delta * latency_sec;
          double v_delay    = v_    + a * latency_sec;
          double cte_delay  = cte_  + v_ * sin(epsi_) * latency_sec;
          double epsi_delay = epsi_ - v_ / Lf * delta * latency_sec;

          // Setup initial state
          Eigen::VectorXd state(6);
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

          // Call to solver
          auto ret_vals = mpc.Solve(state, C);
 
          // Extract actuators, delta and acceleration value
		  double delta_val = ret_vals.end()[-2];
		  double a_val = ret_vals.end()[-1];

          // normalized to -1, 1 and convert to unity convention by *-1
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = delta_val/deg2rad(25) * -1;
          double throttle_value = a_val; 

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory in green color 
          // appending predicted trajectory from solver-returned list
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int t = 0 ; t < N; t++) {
              mpc_x_vals.push_back(ret_vals[t]);
              mpc_y_vals.push_back(ret_vals[t+N]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
   
          // Display the waypoints/reference line in yellow color
          // 60 units in x with delay offset in x
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		  for (int xtick = 0; xtick<60; xtick+=3) {
            double _x = x_delay + xtick;
		  	next_x_vals.push_back(_x);
			next_y_vals.push_back(polyeval(C,_x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

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

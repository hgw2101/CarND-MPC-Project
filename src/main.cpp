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
const double Lf = 2.67;
const double latency_in_ms = 100;

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

// TODO: transform map coordinates to vehicle coordinates
void transform(vector<double>& ptsx, vector<double>& ptsy, double& px, double& py, double& psi) {
  for (int i=0; i<ptsx.size(); i++) {
    // px and py should always be (0,0) in vehicle coordinates, now we just
    // have to convert waypoints to vehicle coordinates
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;

    // use the transformation formula from project 3 for x, 
    // obs.x * cos(theta) - obs.y * sin(theta) + x; and y:
    // obs.x * sin(theta) + obs.y * cos(theta) + y;
    //-psi here because it's vechile to map is positive so vehicle to map should be negative, and no need to add x or y since they're 0 in vehicle coords
    ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi); 
    ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
  }
  // lastly, set px, py and psi to 0
  px = 0;
  py = 0;
  psi = 0; // psi is always 0 in vehicle coordinates
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
    // cout << sdata << endl;
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

          // accounting for latency: predict where the vehicle will be after 100ms using the vehicle model
          px += v * cos(psi) * latency_in_ms/1000;
          py += v * sin(psi) * latency_in_ms/1000;

          double current_throttle = j[1]["throttle"];
          v += current_throttle * latency_in_ms/1000;

          double current_steer_angle = j[1]["steering_angle"];
          psi -= v/Lf * current_steer_angle * latency_in_ms/1000;  //ALMOST FORGOT THIS NEEDS TO BE MINUS AGAIN!!!
          
          // convert points from map to vehicle coordinates
          transform(ptsx, ptsy, px, py, psi);

          Eigen::Map<Eigen::VectorXd> ptsx_eigen(ptsx.data(), ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_eigen(ptsy.data(), ptsy.size());
          
          // *1) calculate coeffs, which would get us the reference trajectory
          auto coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);
          
          // *2) initialize the cte and epsi, and construct the state vector with the 
          // initial values and pass to the solver, the solver will then return a list of
          // state variables, which will be used to plot the path, and control inputs, 
          // which will be used to control the car
          
          // *2a) cte is the diff between the reference y, which is evaluating the polynomial
          //      given x position and the current y value of the vehicle
          double cte = polyeval(coeffs, px) - py; // or simply polyeval(coeffs, 0) since px and py are 0

          // *2b) calculate error of psi, which is the difference between actual psi, and
          //      desired psi, psides, which is angle formed based on the tangent of the
          //      reference trajectory, given by coeffs[1]

          double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px, 2));

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          
          // *3) calculate the initial steer_value and throttle_value using MPC
          double steer_value;
          double throttle_value;

          vector<double> vars = mpc.Solve(state, coeffs);

          // TODO: only grabbing steer_value/delta and throttle_value/acceleration for now, will be grabbing
          // the x and y for visualization later
          steer_value = vars[0];
          throttle_value = vars[1];
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/(deg2rad(25) * Lf); // TODO: still not sure why dividing by deg2rad(25)*Lf
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i=2; i<vars.size(); i++) {
            // every second element represents x value
            if (i%2==0) {
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // this would set the yellow line as waypoints, but we should use the polynomial
          /*
          next_x_vals = ptsx;
          next_y_vals = ptsy;
          */

          double seg_dist = 2.5;
          int segments = 30;
          for (int i=0; i<segments; i++) {
            next_x_vals.push_back(i*seg_dist);
            next_y_vals.push_back(polyeval(coeffs, seg_dist * i));
          }
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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

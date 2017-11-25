#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  pid = PID();
  double pid_weights[] = {1.5e-1, 1e-4, 1.7e3};
  double pid_weight_deltas[] = {pid_weights[0] / 20, pid_weights[1] / 10, pid_weights[2] / 5};
  pid.Init(pid_weights[0], pid_weights[1], pid_weights[2]);

  PID throttle_pid;
  throttle_pid = PID();
  throttle_pid.Init(1e-1, 1e-3, 1e1);

  // variables for self-twiddling code.
  // This would be tidier in a struct/class, but is currently experimental
  int step_count = 0;
  int weights_index = 0;
  int twiddle_stage = 0;
  double previous_max_error = 1e10;
  double max_error = 0;

  h.onMessage([&pid, &throttle_pid, &step_count, &pid_weights, &weights_index, &twiddle_stage, &max_error, &previous_max_error, &pid_weight_deltas](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // Quit if we crash.
          // Useful for making sense of the log
          // if the self-twiddling code is on.
          if (step_count > 100 && speed < 1) exit(0);

          step_count++;
          // Experimental self-twiddling code.
          // It works, but can lead to increased instability
          // as it experiments.
          /*
          //if (fabs(cte) > max_error) max_error = fabs(cte);
          
          max_error += pow(cte, 2);
          if (step_count % 600 == 0) { // approximately one loop
            if (twiddle_stage == 0) {
              if (max_error < previous_max_error) previous_max_error = max_error;
              pid_weights[weights_index] += pid_weight_deltas[weights_index];
              twiddle_stage = 1;

            } else if (twiddle_stage == 1) {
              if (max_error < previous_max_error) {
                previous_max_error = max_error;
                pid_weight_deltas[weights_index] *= 1.1;
                twiddle_stage = 0;
                weights_index++;
                weights_index %= 3;

              } else {
                pid_weights[weights_index] -= 2 * pid_weight_deltas[weights_index];
                twiddle_stage = 2;
              }

            } else if (twiddle_stage == 2) {
              if (max_error < previous_max_error) {
                previous_max_error = max_error;
                pid_weight_deltas[weights_index] *= 1.1;
              } else {
                pid_weights[weights_index] += pid_weight_deltas[weights_index];
                pid_weight_deltas[weights_index] *= 0.9;
              }
              twiddle_stage = 0;
              weights_index++;
              weights_index %= 3;
            }
            
            pid.Init(pid_weights[0], pid_weights[1], pid_weights[2]);
            std::cout << "\n\nWeights: " << pid_weights[0] << ", " << pid_weights[1] << ", " << pid_weights[2] << std::endl;
            std::cout << "Error: " << max_error << " Best error: " << previous_max_error << std::endl;
            max_error = 0;
          }*/

          pid.UpdateError(cte);
          double error = pid.TotalError();

          steer_value = error;

          double throttle_error = 30 - speed;
          throttle_pid.UpdateError(-throttle_error);
          double throttle;
          throttle = throttle_pid.TotalError();
          //throttle = 0.3; // this was the original throttle that came from the project stub
          
          if (steer_value > 0.9)
            steer_value = 0.9;
          if (steer_value < -0.9)
            steer_value = -0.9;
          
          // DEBUG
          std::cout << "Step: " << step_count << " CTE: " << cte << " Steering Value: " << steer_value << " Angle: " << angle << " Throttle: " << throttle <<  std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

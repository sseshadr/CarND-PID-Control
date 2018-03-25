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
  //// Initialize the pid variable to tune.
  //double Kp = 0.1;
  //double Ki = 0.0001;
  //double Kd = 3;
  //pid.Init(Kp, Ki, Kd);

  // Initialize tuned pid variable.
  // Comment this and uncomment previous section to tune.
  double Kp = 0.265;
  double Ki = 0.095;
  double Kd = 2.995;
  pid.Init(Kp, Ki, Kd);
  pid.tuned = 2;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  double throttle = 0.3;

		  // Get PID output
		  pid.idx = pid.idx + 1;
		  pid.UpdateError(cte);
		  steer_value = pid.TotalError();

          // DEBUG
		  //std::cout << "Sample... " << pid.idx << std::endl;
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

		  // If PID not tuned, and simulator reaches defined 'N' # of samples, go twiddle.
		  if (pid.tuned != 2 && pid.idx == pid.N) {

			  // Reset Sim
			  steer_value = 0.0;
			  throttle = 0.0;
			  msgJson["steering_angle"] = steer_value;
			  msgJson["throttle"] = throttle;
			  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			  std::string reset_msg = "42[\"reset\",{}]";
			  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

			  // Record initial error
			  if (pid.tuned == 0) {
				  pid.bestError = pid.currentError;
				  pid.tuned = 1;
				  // Initial output
				  std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
					  << " Initial Error: " << pid.currentError << " Best Error: " << pid.bestError << std::endl;
				  std::cout << "Twiddle starting..." << std::endl;
				  pid.Reset();
			  }
			  else {
				  // Iteration output
				  std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
					  << " Current Error: " << pid.currentError << " Best Error: " << pid.bestError << std::endl;
			  }

			  // Twiddle
			  if (pid.tuned == 1) {
				  pid.Twiddle();
				  pid.Reset();
			  }
		  }
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

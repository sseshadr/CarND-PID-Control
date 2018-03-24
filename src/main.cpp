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

int runSim(PID pid) {

	uWS::Hub h;
	
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
					// double speed = std::stod(j[1]["speed"].get<std::string>());
					// double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;
										
					pid.idx = pid.idx + 1;
					pid.UpdateError(cte);
					steer_value = (-pid.p[0] * pid.p_error) + (-pid.p[1] * pid.i_error) + (-pid.p[2] * pid.d_error);

					if (steer_value >= 1.0) {
						steer_value = 1.0;
						pid.windup = 1.0;
					}
					else if (steer_value <= -1.0) {
						steer_value = -1.0;
						pid.windup = 1.0;
					}

					std::cout << pid.idx << std::endl;
					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = 0.3;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

					if (pid.idx == pid.N) {
						json msgJson;
						msgJson["steering_angle"] = 0;
						msgJson["throttle"] = 0;
						auto msg = "42[\"steer\"," + msgJson.dump() + "]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

						std::string reset_msg = "42[\"reset\",{}]";
						ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
						pid.idx = 0;
						pid.currentError = pid.currentError / pid.N;
						//NEED TO GET OUT HERE!
					}
				}
			}
			else {
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

int main()
{
  PID pid;

  // Initialize the pid variable.
  double Kp = 0.1;
  double Ki = 0.0001;
  double Kd = 3;
  pid.Init(Kp, Ki, Kd);

  // Get inital error from sim
  std::cout << "Running Initial Sim... " << std::endl;
  int status = runSim(pid);

  if (pid.tuned == 0) {
	  pid.bestError = pid.currentError;
	  pid.tuned = 1;
	  std::cout << "Beginning twiddle.." << std::endl;
  }
  std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
	  << " Initial Error: " << pid.currentError << " Initial Best Error: " << pid.bestError << std::endl;

 
  while ((pid.dp[0] + pid.dp[1] + pid.dp[2]) > pid.tol) {

  		for (int idx = 0; idx < 3; idx++) {
  			std::cout << "Twiddling up.." << std::endl;
  			pid.p[idx] += pid.dp[idx];

			status = runSim(pid);
			std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
				<< " Current Error: " << pid.currentError << " Best Error: " << pid.bestError << std::endl;
  		
  			if (pid.currentError < pid.bestError) {
  				pid.bestError = pid.currentError;
  				pid.dp[idx] *= 1.1;
  			}
  			else {
  				std::cout << "Twiddling down.." << std::endl;
  				pid.p[idx] -= 2 * pid.dp[idx];

				status = runSim(pid);
				std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
					<< " Current Error: " << pid.currentError << " Best Error: " << pid.bestError << std::endl;

  				if (pid.currentError < pid.bestError) {
  					pid.bestError = pid.currentError;
  					pid.dp[idx] *= 1.1;
  				}
  				else {
  					pid.p[idx] += pid.dp[idx];
  					pid.dp[idx] *= 0.9;
  				}
  			}
  		}
	}
  pid.tuned = 2;
  pid.N = 5000;
  std::cout << "Running Final Sim... " << std::endl;
  runSim(pid);
  std::cout << "Kp: " << pid.p[0] << " Ki: " << pid.p[1] << " Kd: " << pid.p[2]
	  << " Final Error: " << pid.currentError << " Final Best Error: " << pid.bestError << std::endl;
  return 0;
}

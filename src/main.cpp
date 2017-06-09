#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define TWIDDLE_TUNE 0
#define DEBUG 0

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
  // TODO: Initialize the pid variable.
  if(TWIDDLE_TUNE)
  {
    //perform parameter tuning
    pid.Init(0.01, 1.0, 0.001); //0.1,3.0,0.001
  }
  else
  {
    //directly use results from parameter tuning
    //best params: 0.341, 4.31, 0.0 
    pid.Init(0.341, 4.31, 0.001); 
  } 
  long count = 0;
  //pid.twiddleFlag = true;
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") 
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") 
        {
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
          if(TWIDDLE_TUNE)
          {
            if(pid.computeTwiddleErrFlag)
            {
              //twiddle mode
              if(DEBUG)
              {
                std::cout << "Computing initial twiddle error "  << std::endl;
              }
              pid.twiddleIterCount += 1;
              if(pid.twiddleIterCount >= pid.minSteps)
              {
                pid.twiddleErr += cte * cte;
              }
              if(pid.twiddleIterCount == pid.totalSteps)
              {
                pid.twiddleIterCount = 0;
                pid.twiddleErr /= (pid.totalSteps - pid.minSteps);
                if(DEBUG)
                {
                    std::cout << "Initial error: " << pid.twiddleErr << std::endl;
                }
                pid.computeTwiddleErrFlag = false;
                pid.twiddleFlag = true;
                //Initialize final error
                pid.finalErr = pid.twiddleErr;

                //Step up twiddle params
                pid.Kp += pid.dp[0];
                pid.Kd += pid.dp[1];
                pid.Ki += pid.dp[2];

                //reset simulator
                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }

            }

            if(pid.twiddleFlag)
            {
              if(DEBUG)
              {
	              std::cout << "Performing twiddle " << std::endl;
	              std::cout << "Kp " << pid.Kp << "\t" << "Kd " << pid.Kd << "\t" << "Ki " << pid.Ki << std::endl; 
              }
              
              pid.twiddleIterCount += 1;
              if(pid.twiddleIterCount == 1)
              {
                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }

              
              
              if(pid.finalErr < 0.05)
              {
                pid.twiddleFlag = false;
                pid.Kp = pid.bestKp;
                pid.Kd = pid.bestKd;
                pid.Ki = pid.bestKi;

                std::cout << "Final Error " << pid.finalErr << std::endl;
                std::cout << "Best Kp " << pid.Kp << "\t" << "Best Kd " << pid.Kd << "\t" << "Best Ki " << pid.Ki << std::endl; 
                //reset simulator
                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  			  }
              else
              {
                pid.Twiddle(cte, pid.twiddleErr, pid.dp);
                double currUpdateErr = pid.updateErr/ (pid.twiddleIterCount - pid.minSteps); 
                if(DEBUG)
                {
	                std::cout << "dp1 " << pid.dp[0] << "\t" << "dp2 " << pid.dp[1] << "\t" << "dp3 " << pid.dp[2] << std::endl;
	                std::cout << "Best error: " << pid.twiddleErr << "\t" << "Update Error: " << currUpdateErr << std::endl;
	                std::cout << "dp index " << pid.pIndex << std::endl;
                }
              }

              if(pid.twiddleErr < pid.finalErr)
              {
                pid.finalErr = pid.twiddleErr;
                pid.bestKp = pid.Kp;
                pid.bestKd = pid.Kd;
                pid.bestKi = pid.Ki;
              }

            }
          }

          //happy mode
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          steer_value = (steer_value < -1.0) ? -1.0: steer_value;
          steer_value = (steer_value > 1.0) ? 1.0: steer_value;

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);  
        }
      } 
      else 
      {
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

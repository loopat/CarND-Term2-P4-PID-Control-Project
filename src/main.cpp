#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

//#define _TWIDDLE_
#define COUNTER_THRESHOLD (1200u)

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

  for(int i = 0; i < 3; i ++)
  {
     pid.p[i] = 0.0;
     pid.dp[i] = 1.0;
  }
    
  pid.counter = 0;
  pid.acc_err = 0;
  pid.bDecreseFlag = 0;
  pid.bFirstStep = 0;
  pid.i = 0;
  pid.best_Kp = 0;
  pid.best_Ki = 0;
  pid.best_Kd = 0;

  pid.Init(pid.p[0],pid.p[1],pid.p[2]);
    
  //pid.Init(0.2,0.0004,2.0);
  //pid.Init(0.1,0.00001,1.8);
    
#ifndef _TWIDDLE_
   pid.Init(0.08, 0.00001, 1.84);
#endif
    
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      /* more than one circle */
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
#ifdef _TWIDDLE_
                    /* reset position */
                    if(pid.counter > COUNTER_THRESHOLD )//|| pid.acc_err > 1000)
                    {
                        /* equal while */
                        if((pid.dp[0] + pid.dp[1] + pid.dp[2]) < 0.01)
                        {
                            std::cout << "The final parameters are : " << std::endl;
                            std::cout << "P : "<< pid.best_Kp << ", I: " << pid.best_Ki << ", D : " << pid.best_Kd << std::endl;
                            while(1);
                        }

                        /* Init best error, only executed once */
                        if(0 == pid.bFirstStep)
                        {
                            pid.best_err = pid.acc_err;
                            pid.bFirstStep = 1;
                            pid.i = 0;
                            pid.p[pid.i] += pid.dp[pid.i];
                        } else if(1 == pid.bDecreseFlag)
                        {
                            if(pid.acc_err < pid.best_err)
                            {
                                pid.best_err = pid.acc_err;
                                pid.dp[pid.i] *= 1.05;
                                pid.best_Kp = pid.p[0];
                                pid.best_Ki = pid.p[1];
                                pid.best_Kd = pid.p[2];
                            } else
                            {
                                pid.p[pid.i] += pid.dp[pid.i];
                                pid.dp[pid.i] *= 0.95;
                            }
                            pid.i = (pid.i + 1) % 3;
                            pid.p[pid.i] += pid.dp[pid.i];
                            pid.bDecreseFlag = 0;
                        }
                        else
                        {
                            if(pid.acc_err < pid.best_err)
                            {
                                pid.best_err = pid.acc_err;
                                pid.dp[pid.i] *= 1.1;
                                pid.best_Kp = pid.p[0];
                                pid.best_Ki = pid.p[1];
                                pid.best_Kd = pid.p[2];
                                /* iterate i */
                                pid.i = (pid.i + 1) % 3;
                                pid.p[pid.i] += pid.dp[pid.i];
                            } else
                            {
                                pid.p[pid.i] -= 2.0 * pid.dp[pid.i];
                                pid.bDecreseFlag = 1;
                                /* DO NOT iterat i */
                            }
                        }

                        pid.counter = 0;
                        pid.acc_err = 0.0;

                        pid.Init(pid.p[0],pid.p[1],pid.p[2]);

                        std::string msg = "42[\"reset\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
#endif
                    std::cout << "P : "<< pid.p[0] << ", I: " << pid.p[1] << ", D : " << pid.p[2] << std::endl;
                    std::cout << "dp[0] : "<< pid.dp[0] << ",dp[1] : " << pid.dp[1] << ",dp[2] : " << pid.dp[2] << std::endl;

                    pid.acc_err += cte * cte;
                    std::cout << "Speed is :" << speed << std::endl;
#ifdef _TWIDDLE_
                    /* If the car stop, add a large error */
                    if((speed < 1))
                    {
                        if (pid.counter > 300)
                        {
                            pid.acc_err += 100;
                        }
                    }
                    
                    /* stop this try if it has exceed the best error much */
                    if( (1 == pid.bFirstStep) && (pid.acc_err > pid.best_err))
                    {
                        pid.counter = COUNTER_THRESHOLD;
                    }
#endif
                    pid.counter += 1;
                    
                  pid.UpdateError(cte);
                  steer_value = pid.TotalError();
                   
                  std::cout << "the index of i is :" << pid.i << std::endl;
                  std::cout << "counter is : " << pid.counter << std::endl;
                  std::cout << "acculate err is : " << pid.acc_err << std::endl;
                  std::cout << "best error is : " << pid.best_err << std::endl;
                  std::cout << "best Kp is : " << pid.best_Kp << ", best Ki is : " << pid.best_Ki << ", best Kd is : " << pid.best_Kd << std::endl;
                  
                  // DEBUG
                  std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                  json msgJson;
                  msgJson["steering_angle"] = steer_value;
                  msgJson["throttle"] = 0.3;
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

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "tools.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace Eigen;
using namespace std;

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

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        auto time = chrono::steady_clock::now();
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (!s.empty()) {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    AbsoluteWaypoints trackAbsolute = {j[1]["ptsx"], j[1]["ptsy"]};
                    double px = j[1]["x"],
                            py = j[1]["y"],
                            psi = j[1]["psi"],
                            v = j[1]["speed"],
                            delta = j[1]["steering_angle"],
                            a = j[1]["throttle"];

                    // modify data: we know we have latency, so the expected position of the vehicle will change
                    const double expectedLatency = 0.0;
                    px += v * cos(psi) * expectedLatency;
                    py += v * sin(psi) * expectedLatency;
                    psi += v / Lf * delta * expectedLatency;
                    v += a * expectedLatency;

                    // convert absolute waypoints to relative ones using the vehicle's location and orientation
                    RelativeWaypoints trackWaypoints = transformToRelative(trackAbsolute, {px, py}, psi);

                    // fit a 3rd order polynomial
                    VectorXd coeffs = polyfit(vecXd(trackWaypoints.x), vecXd(trackWaypoints.y), 3);


                    // now we build our state which consists of 6 values: x, y, psi, v and error values cte end epsi
                    // remember that values x, y and psi are now zero because we translated the coordinates to the
                    //  car's position and orientation!
                    double cte = polyeval(coeffs, 0);
                    double epsi = -(atan(polyeval(derivative(coeffs), 0)));
                    VectorXd state = VectorXd::Zero(6);
                    state << 0, 0, 0, v, cte, epsi;


                    // use MPC to solve
                    const MPCSolution &solution = mpc.Solve(state, coeffs);

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    // NOTE: While our MPC model is using the accurate vehicle model where positive angle means to steer
                    // to the left, in the Udacity CarND simulator a positive angle means to steer to the right.
                    // So we just flip the angle here!
                    msgJson["steering_angle"] = -solution.steering_delta / deg2rad25;
                    msgJson["throttle"] = solution.acceleration;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    // display MPC predictions line (green)
                    msgJson["mpc_x"] = solution.waypoints.x;
                    msgJson["mpc_y"] = solution.waypoints.y;

                    // display the reference line (yellow)
                    msgJson["next_x"] = trackWaypoints.x;
                    msgJson["next_y"] = trackWaypoints.y;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    cout << msg << endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    // TODO: enable sleep again
//                    this_thread::sleep_for(chrono::milliseconds(100));

                    double latency = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time)
                                             .count() / 1000.0;
                    cout << "latency: " << fixed << setprecision(3) << latency
                         << " seconds, diff from expected latency: " << expectedLatency - latency << endl;

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        cout << "Connected!!!" << endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        cout << "Disconnected" << endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        cout << "Listening to port " << port << endl;
    } else {
        cerr << "Failed to listen to port" << endl;
        return -1;
    }
    h.run();
}

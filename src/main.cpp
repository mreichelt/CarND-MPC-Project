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
                    * TODO: Calculate steering angle and throttle using MPC.
                    *
                    * Both are in between [-1, 1].
                    *
                    */
                    // first we fit a 3rd order polynomial
                    VectorXd coeffs = polyfit(vecXd(ptsx), vecXd(ptsy), 3);
                    cout << "Polynom: y = " << coeffs[0] << " + " << coeffs[1] << " * x + " << coeffs[2] << " * x^2 + "
                         << coeffs[3] << " * x^3" << endl;

                    double cte = polyeval(coeffs, px) - py;
                    // psi minus derivative of 3rd degree polynomial
                    // TODO: add epsi!
//                    double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * px * pow(coeffs[3], 2));
                    double epsi = 0;
                    cout << "CTE = " << cte << ", epsi = " << epsi << endl;


                    // now we build our state which consists of 6 values: x, y, psi, v and error values cte end epsi
                    VectorXd state = VectorXd::Zero(6);
                    state <<
                          px,
                            py,
                            psi,
                            v,
                            cte,
                            epsi;

                    // use MPC to solve
                    vector<double>
                            x_vals = {state[0]},
                            y_vals = {state[1]},
                            psi_vals = {state[2]},
                            v_vals = {state[3]},
                            cte_vals = {state[4]},
                            epsi_vals = {state[5]},
                            delta_vals = {},
                            a_vals = {};

                    int n_predictions_visualization = 3;
                    for (size_t i = 1; i < n_predictions_visualization; i++) {
                        auto vars = mpc.Solve(state, coeffs);

                        x_vals.push_back(vars[0]);
                        y_vals.push_back(vars[1]);
                        psi_vals.push_back(vars[2]);
                        v_vals.push_back(vars[3]);
                        cte_vals.push_back(vars[4]);
                        epsi_vals.push_back(vars[5]);
                        delta_vals.push_back(vars[6]);
                        a_vals.push_back(vars[7]);

                        // re-use predictions from MPC as next state to predict the following one
                        state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
                    }

                    double steer_value = delta_vals.front();
                    // TODO: why negative?
                    double throttle_value = -a_vals.front();
//                    double steer_value = -0.004;
//                    double throttle_value = 0.1;


                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value / deg2rad25;
                    msgJson["throttle"] = throttle_value;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = x_vals;
                    msgJson["mpc_y"] = y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    cout << msg << endl;
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

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
using namespace std;

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
	auto b2 = s.find_last_of("]");
	if (found_null != string::npos) return "";
	else if (b1 != string::npos && b2 != string::npos) return s.substr(b1, b2 - b1 + 1);
	return "";
}

int main(){
	uWS::Hub h;

	PID steering_pid;
	PID throttle_pid;

	// Initialize the Steering PID controller.
	//steering_pid.Init(0.12, 0.0, 1., 0.012, 0.0, .1); // Initial
	steering_pid.Init(0.108, 0.0, 1.1, 0.0, 0.0, .0); // 1% @ 20mph
	//steering_pid.Init(0.123, 0.0, 1.08, 0.0, 0.0, .0); // 1% @ 120mph

	// Initialize the Throttle PID Controller
	//throttle_pid.Init(.1, 0.0, 1., 0.01, 0.0, .1); // Initial
	throttle_pid.Init(.1, 0.0, 1.1, 0.0, 0.0, .0); // 1% @ 20mph
	//throttle_pid.Init(.1, 0.0, 1.19, 0.0, 0.0, .0); // 1% @ 120mph

	int twiddle = 0; // 0 = Off, 1 = Steering, 2 = Throttle, 3 = Both
	double target_speed = 20.;
	double top_speed = 0.;
	double tolerance = .001;
	int icount = 0;
	int this_twid = 0;
	if (twiddle == 3) this_twid = 1;

	h.onMessage([&steering_pid, &throttle_pid, &twiddle, &top_speed, &target_speed, &tolerance, &icount, 
		&this_twid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = stod(j[1]["cte"].get<string>());
					double speed = stod(j[1]["speed"].get<string>());
					double angle = stod(j[1]["steering_angle"].get<string>());
					string red("\n\u001b[38;5;196m");
					string norm("\x1B[0m");
					if (icount == 0) cout << red << "Target Speed: " << target_speed << norm;
					icount++;
					if (speed > top_speed) top_speed = speed;
					// Calculate Steering Value
					steering_pid.UpdateError(cte);
					double steering_value = steering_pid.output();
					// Calculate throttle value
					double speed_error = speed - target_speed; // 20mph
					//double speed_error = speed + fabs(cte) * 7. * speed - target_speed; // 120mph
					if (speed <= 15.) {
						speed_error = speed - target_speed;
						if (fabs(cte) > 1.) steering_value *= 1.25;
					}
					throttle_pid.UpdateError(speed_error);
					double throttle_value = throttle_pid.output();
					// DEBUG
					// cout << "\n CTE: " << cte << " Steering Value: " << steering_value << 
				  //  " Throttle Value: " << throttle_value << " Speed: " << speed << endl;
					json msgJson;
					msgJson["steering_angle"] = steering_value;
					msgJson["throttle"] = throttle_value;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					// cout << msg << endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

					double sum_dps, sum_dpt;
					string pfmt = "%d, #%d, BE %.03f, TS %.01f Sdp %.02f (Kp, Ki, Kd) ";
					pfmt += "(%.03f, %.01f, %.02f) (dpp, dpi, dpd) (%.04f, %.01f, %.03f)";
					if (twiddle > 0 && icount % steering_pid.count_threshold * 2 == 0) {
						if (this_twid == 0) this_twid = twiddle;
						if (this_twid % 2 == 1) {
							sum_dps = steering_pid.dp_p + steering_pid.dp_d + steering_pid.dp_i;
							if (sum_dps > tolerance) {
								steering_pid.twiddle();
								if (steering_pid.print_vars) {
									string cyan("\n\033[22;36m");
									printf((cyan + "S: " + pfmt + norm).c_str(),
										icount, steering_pid.iteration, steering_pid.best_error, top_speed,
										sum_dps, steering_pid.Kp, steering_pid.Ki, steering_pid.Kd, 
										steering_pid.dp_p, steering_pid.dp_i, steering_pid.dp_d);
									if (twiddle == 3) this_twid = -this_twid;
								}
							}
						}
						if (this_twid % 2 == 0) {
							sum_dpt = throttle_pid.dp_p + throttle_pid.dp_d + throttle_pid.dp_i;
							if (sum_dpt > tolerance) {
								throttle_pid.twiddle();
								if (throttle_pid.print_vars) {
									string orange("\n\u001b[38;5;208m");
									printf((orange + "T: " + pfmt + norm).c_str(),
										icount, throttle_pid.iteration, throttle_pid.best_error, top_speed,
										sum_dpt, throttle_pid.Kp, throttle_pid.Ki, throttle_pid.Kd, 
										throttle_pid.dp_p, throttle_pid.dp_i, throttle_pid.dp_d);
									if (twiddle == 3) this_twid = 1;
								}
							}
						}
						if (this_twid < 0) this_twid = -this_twid + 1;
						int iter = throttle_pid.iteration;
						if (twiddle == 1) iter = steering_pid.iteration;
						bool p_vars = throttle_pid.print_vars;
						if (twiddle == 1) p_vars = steering_pid.print_vars;
						int size = 50;
						//cout << "\nI: " << iter << " I%S: " << iter % size << "P: " << p_vars;
						if (iter > 0 && iter % size == 0 && p_vars) {
							target_speed += 2;
							steering_pid.current_state = 0;
							throttle_pid.current_state = 0;
							cout << red << "Target Speed: " << target_speed << norm;
							steering_pid.print_vars = false;
							throttle_pid.print_vars = false;
						}
						fflush(stdout);
						top_speed = 0.;
						steering_pid.total_error = 0.;
						throttle_pid.total_error = 0.;
					}
				}
			}
			else {
				// Manual driving
				string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const string s = "<h1>Hello world!</h1>";
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
		cout << "Connected!!!" << endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		cout << "Disconnected" << endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		cout << "\nListening to port " << port << endl;
	}
	else
	{
		cerr << "Failed to listen to port" << endl;
		return -1;
	}
	h.run();
}
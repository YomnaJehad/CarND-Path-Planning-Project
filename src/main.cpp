#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <limits>
#include <string>
//#include "PathPlanner.h"
using namespace std;

#define LANE_WIDTH 4
#define INEFFICIENCY_COST 10
#define DISTANCE_COST 100

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

struct lane_speed_sort
{
    template<class T>
    bool operator()(T const &a, T const &b) const { return a[0] < b[0]; }
};


// This function gets the speed of the lane (to be used in the inefficiency function). The speed of the lane
// is determined by the speed of the first car whose s is greater than the s of our car.
double lane_speed(int s, vector<vector <double>> sensor_fusion, int lane)
{
	int max_distance_limit = 50;
	vector<vector<double>> lane_speeds;
	for(auto &object: sensor_fusion)
	{
		double d = object[6];
		if(d < 2 + 4*lane + 2.00 && d > 2 + 4 * lane - 2.00)
		{
			double vx = object[3];
			double vy = object[4];
			double check_speed = sqrt(vx*vx + vy*vy);
			lane_speeds.push_back({object[5], check_speed});
		}
	}
	sort(lane_speeds.begin(), lane_speeds.end(), lane_speed_sort());
	for(auto &speed: lane_speeds)
	{
		if(speed[0] > s && speed[0] < s + max_distance_limit)
		{
			return speed[1];
		}
	}
	return 50;
}

// This function gets the efficiency cost of the lane. the higher the lane speed, the lower the cost.
double inefficiency_cost(int s, vector<vector <double>> sensor_fusion, int lane)
{
	double speed = lane_speed(s, sensor_fusion, lane);
	return (abs(speed - 50.0) / 50.0);
}

// returns the lane number corresponding to a certain d coordinate.
int get_lane(double d)
{
	return (int)(d / 4);
}

// Returns the states of the car in order to search in them.
vector<int> possible_states(int lane) {
    /*
    Provides the possible next states given the current state for the FSM
	the car has 3 states representing 3 lanes on the road.
    */
    vector<int> states;
    states.push_back(lane);
    //string state = this->state;
    if(lane == 0)
    {
    	states.push_back(1);
    }
    else if(lane == 1)
	{
    	states.push_back(0);
    	states.push_back(2);
	}
    else
    {
    	states.push_back(1);
    }

    return states;
}

// This function makes predictions about the other vehicles s values at the end of our car's trajectory.
vector<double> make_predictions(int prev_size, double car_s, vector<vector <double>> sensor_fusion)
{
	vector<double>predictions;
	for(auto &object : sensor_fusion)
	{
		double vx = object[3];
		double vy = object[4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = object[5];

		check_car_s += ((double)prev_size * 0.02 * check_speed);
		predictions.push_back(check_car_s);
	}
	return predictions;
}

//This function gets the cost associated with how far, or close, other vehicles are with respect to our car.
double distance_cost(int s, vector<vector <double>> sensor_fusion, vector <double> predictions,int lane)
{
	int max_distance_limit = 100;
	vector<double> lane_distances;
	for(size_t i = 0; i < sensor_fusion.size(); ++i)
	{
		double d = sensor_fusion[i][6];
		if(d < 2 + 4*lane + 2.00 && d > 2 + 4 * lane - 2.00)
		{
			lane_distances.push_back(predictions[i]);
		}
	}
	for(auto &distance: lane_distances)
	{
		if(abs(distance  - s ) < 25)
		{
			double cost = 1;
			return cost;
		}
	}
	return 0;
}

// This function calculates to total cost of the car being in some lane and returns it.
// Distance costs which represent safety costs are heavily weighted more than the efficiency(speed) cost
double calculate_cost(double car_s, int state, vector<double> predictions, vector<vector <double>> sensor_fusion)
{
	double cost = 0;
	vector<double> costs;
	vector<double> weights = {INEFFICIENCY_COST, DISTANCE_COST};

	double ineff_cost = inefficiency_cost(car_s, sensor_fusion, state);
	costs.push_back(ineff_cost);

	double dist_cost = distance_cost(car_s,sensor_fusion, predictions, state);
	costs.push_back(dist_cost);

	for(size_t i = 0; i < costs.size(); ++i)
	{
		cost += (double)weights[i] * costs[i];
	}
	return cost;
}

// This function chooses the next state(lane) for the car. It choses the lane with the lowest cost.
double choose_next_state(int lane, int prev_size, double car_s, vector<vector <double>> sensor_fusion)
{

	vector<int> states = possible_states(lane);
    vector<double> predictions = make_predictions(prev_size, car_s, sensor_fusion);
    double cost = std::numeric_limits<double>::max();
    int selected_state = lane;

    for(auto &state : states)
    {
    	double state_cost = calculate_cost(car_s, state , predictions, sensor_fusion);

    	// makes sure the lane is safe to drive in and there is an actual speed difference
    	//between the lanes and not just noise
        if(cost - state_cost > 4 && state_cost < 10)
        {
        	cost = state_cost;
            selected_state = state;
        }
    }
    return selected_state;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  double ref_vel = 0; //mph
  int lane = 1;
  int starting_counter = 0;
  h.onMessage([&lane,&starting_counter,
			   &ref_vel,
			   &map_waypoints_x,
			   &map_waypoints_y,
			   &map_waypoints_s,
			   &map_waypoints_dx,
			   &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws,
			   char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	double dist_inc = 0.3;;
        	int prev_size = previous_path_x.size();
        	bool too_close = false;
        	if(prev_size > 0)
        	{
        		car_s = end_path_s;
        	}
        	// The car is not allowed to change lanes under 35 MPH speed. It should perform changes quickly.
        	// The car should also be in the middle of the lane to be able to make changes. It cannot
        	// choose another lane while it's already making lane changes
        	if(car_speed > 35 && car_d < 2.5 + (lane * 4) && car_d > 1.5 + (lane * 4))
        	{
        		lane = choose_next_state(get_lane(car_d), prev_size, car_s, sensor_fusion);
        	}


        	// checks to see if there is a car at 30 miles near our car. If so our car slows down
        	for(auto &object : sensor_fusion)
        	{
        		double d = object[6];
        		if(d < 2 + 4*lane + 2.00 && d > 2 + 4 * lane - 2.00)
        		{
        			double vx = object[3];
        			double vy = object[4];
        			double check_speed = sqrt(vx*vx + vy*vy);
        			double check_car_s = object[5];

        			check_car_s += ((double)prev_size * 0.02 * check_speed);

        			if(check_car_s > car_s && (check_car_s - car_s) < 30)
        			{
        				too_close = true;
        			}
        		}
        	}
        	if(too_close)
        	{
        		ref_vel -= 0.3	;
        	}
        	else if(ref_vel < 48.5)
        	{
        		ref_vel += 0.224;
        		too_close = false;
        	}

        	vector<double> ptsx;
        	vector<double> ptsy;

        	double ref_x = car_x;
        	double ref_y = car_y;
        	double ref_yaw = deg2rad(car_yaw);

        	if ( prev_size < 2 )
        	{
        	    double prev_car_x = car_x - cos(car_yaw);
        	    double prev_car_y = car_y - sin(car_yaw);

        	    ptsx.push_back(prev_car_x);
        	    ptsx.push_back(car_x);

        	    ptsy.push_back(prev_car_y);
        	    ptsy.push_back(car_y);
        	}
        	else
        	{
        		ref_x = previous_path_x[prev_size - 1];
        	    ref_y = previous_path_y[prev_size - 1];

        	    double ref_x_prev = previous_path_x[prev_size - 2];
        	    double ref_y_prev = previous_path_y[prev_size - 2];
        	    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        	    ptsx.push_back(ref_x_prev);
        	    ptsx.push_back(ref_x);

        	    ptsy.push_back(ref_y_prev);
        	    ptsy.push_back(ref_y);
        	}



            vector<double> next_wp0 = getXY(car_s+30, LANE_WIDTH/2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, LANE_WIDTH/2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, LANE_WIDTH/2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for (size_t i = 0; i < ptsx.size(); i++ )
            {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }


            // Create the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            next_x_vals.clear();
            next_y_vals.clear();

            for ( int i = 0; i < prev_size; i++ ) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate distance y position on 30 m ahead.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - prev_size; i++ ) {

                // 2.24 constant is to convert miles per hours to meter per seconds
                double N = target_dist/(0.02*ref_vel/2.24);
                double x_point = x_add_on + target_x/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // rotate back to normal after rotating it earlier
                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

          	// END OF MY CODE.
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
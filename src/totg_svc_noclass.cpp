/*
Time-Optimal Path Following (July 2012)
We introduce a novel method to generate the time-optimal trajectory that exactly follows a given 
differentiable joint-space path within given bounds on joint accelerations and velocities. We also
present a path preprocessing method to make nondifferentiable paths differentiable by adding 
circular blends. We introduce improvements to existing work that make the algorithm more robust in 
the presence of numerical inaccuracies. Furthermore we validate our methods on hundreds of
randomly generated test cases on simulated and real 7-DOF robot arms. Finally, we provide open 
source software that implements our algorithms.

http://www.golems.org/papers/KunzRSS12-Trajectories.pdf
*/

#include <iostream>
#include <Eigen/Core>
#include <trajectories-totg/Trajectory.h>
#include <trajectories-totg/Path.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <iris_support_msgs/IrisJSONsrv.h>
#include <nlohmann/json.hpp>

using namespace std;
using namespace Eigen;

bool totg_svc_cb(iris_support_msgs::IrisJSONsrvRequest &req, iris_support_msgs::IrisJSONsrvResponse &res)
{
    list<VectorXd> waypoints;
    
    ROS_INFO("[TOTG] Request received, parsing...");
    //TODO: handle shitty json
    nlohmann::json input = nlohmann::json::parse(req.json_str);
    
    // Load waypoints into eigen type
    int n_points = input["points"].size();
    for(int i=0; i<n_points; i++)
    {
        std::vector<double> inpoint = input["points"][i];
        Map<VectorXd> point(inpoint.data(), inpoint.size());

        waypoints.push_back(point);

    }
    // load constraints into eigen types
    std::vector<double> qdmax = input["qdmax"];
    std::vector<double> qddmax = input["qddmax"];
    Map<VectorXd> maxVelocity(qdmax.data(), qdmax.size());
    Map<VectorXd> maxAcceleration(qddmax.data(), qddmax.size());

    ROS_INFO("[TOTG] Received %ld wps in request, Planning...", waypoints.size());

    // do the work.
    auto path = Path(waypoints, 0.1);
    Trajectory trajectory(path, maxVelocity, maxAcceleration);

    ROS_INFO("[TOTG] Plan complete.");
    
    std::vector<std::vector<double>> points, vels;

    // check output, and sample if it's good.
    nlohmann::json output;
    if(trajectory.isValid()) {
        double duration = trajectory.getDuration();
        double dt = 0.1;
        ROS_INFO("[TOTG] Valid trajectory calculated, duration: %f, sampling at dt = %f", duration, dt);
        std::vector<std::vector<double>> points, vels;
        std::vector<double> times;
        for(double t = 0.0; t <= duration; t += dt) {
            VectorXd position = trajectory.getPosition(t);
            VectorXd velocity = trajectory.getVelocity(t);
            std::vector<double> point(position.data(), position.data()+position.size());
            std::vector<double> vel(velocity.data(), velocity.data()+velocity.size());
            times.push_back(t);
            points.push_back(point);
            vels.push_back(vel);
        }
        output["times"] = times;
        output["points"] = points;
        output["vels"] = vels;
        output["success"] = true;
        output["message"] = "Completed.";
        ROS_INFO("[TOTG] Sampling complete, returning %ld points.", points.size());
    }
    else {
        ROS_ERROR("[TOTG] trajectory generation failed.");
        output["success"] = false;
        output["message"] = "failed";
    }
    // const_cast<std::string&>(svc_res_str) = output.dump();
    res.json_str = output.dump();

    return(true);
}



int main(int argc, char** argv) {
	list<VectorXd> waypoints;
    
    ros::init(argc, argv, "totg");
    ros::NodeHandle nh;
    ros::ServiceServer totg_svc = nh.advertiseService("/IRIS/backends/trajectory/hardcoded_totg", totg_svc_cb);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(50.0);
    // main loop
    while(ros::ok())
    {
        rate.sleep();
    }

	return 0;
}



#include <trajectories-totg/Trajectory.h>
#include <trajectories-totg/Path.h>

#include <nlohmann/json.hpp>
#include <Eigen/Core>

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <iris_support_msgs/IrisJSONsrv.h>


#include <fstream>
#include <sstream>
#include <cstdio>
#include <ros/package.h>


using namespace std;
using namespace Eigen;



vector<string> split(const string &s, char delim) {
    vector<string> result;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

class totg
{
private:
    ros::ServiceServer totg_svc;
    ros::NodeHandle nh_;
    int num_dof=6;          //TODO
    
    //totg things.
    VectorXd maxAcceleration_;
    VectorXd maxVelocity_;

public:
    totg(ros::NodeHandle &nh):
    nh_(nh)
    {

        // init
        maxAcceleration_.resize(num_dof);
        maxVelocity_.resize(num_dof);

        //populate
        maxVelocity_ << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        maxAcceleration_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        totg_svc = nh_.advertiseService("/IRIS/backends/trajectory/hardcoded_totg", &totg::totg_svc_cb, this);
    }


    bool totg_svc_cb(iris_support_msgs::IrisJSONsrvRequest &req, iris_support_msgs::IrisJSONsrvResponse &res)
    {
        double maxdeviation = 0.1;
        list<VectorXd> waypoints;

        ROS_INFO("[TOTG] TOTG request received, parsing...");
        nlohmann::json input = nlohmann::json::parse(req.json_str);
        
        int n_points = input["points"].size();
        for(int i=0; i<n_points; i++)
        {
            std::vector<double> inpoint = input["points"][i];
            Map<VectorXd> point(inpoint.data(), 6);

            waypoints.push_back(point);

        }
        ROS_INFO("wps");
        for(VectorXd el:waypoints)
        {
            std::cout << "\n" << el << std::endl;
        }

        ROS_INFO("[TOTG] TOTG planning step 1...");
        auto path = Path(waypoints, maxdeviation);
        ROS_INFO("[TOTG] TOTG planning step 2...");
        Trajectory trajectory(path, maxVelocity_, maxAcceleration_);
        ROS_INFO("[TOTG] TOTG planning step 3...");
        // trajectory.outputPhasePlaneTrajectory();        // writes to file
        ROS_INFO("[TOTG] TOTG plan completed.");
        nlohmann::json output;
        if(!trajectory.isValid())
        {
            ROS_ERROR("[TOTG] Trajectory was not valid.");
            output["message"] = "Trajectory was not valid.";
            output["success"] = false;
            res.json_str = output.dump();
            return(false);
        }

        ROS_INFO("[TOTG] Trajectory was valid.");
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
        res.json_str = output.dump();

        return(true);
    }


};


int main(int argc, char** argv) {

    ros::init(argc, argv, "totg");
    ros::NodeHandle nh;
	list<VectorXd> waypoints;
    ROS_INFO("[TOTG] TOTG starting...");
    
    ROS_FATAL("THIS NODE LOCKS UP WHEN TRYING TO INTERPOLATE.");
    
    totg planner(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(20.0);
    ROS_INFO("[TOTG] TOTG Ready!");
    while(ros::ok())
    {
        // ros::spinOnce();
        rate.sleep();
    }

	return 0;
}



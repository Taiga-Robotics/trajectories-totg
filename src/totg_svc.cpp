#include <trajectories-totg/Trajectory.h>
#include <trajectories-totg/Path.h>

#include <nlohmann/json.hpp>
#include <Eigen/Core>

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <iris_support_msgs/IrisJSONsrv.h>


using namespace std;
using namespace Eigen;


/*
    service handler
*/


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
        maxAcceleration_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        totg_svc = nh_.advertiseService("totg", &totg::totg_svc_cb, this);
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
        ROS_INFO("[TOTG] TOTG planning...");
        Trajectory trajectory(Path(waypoints, maxdeviation), maxVelocity_, maxAcceleration_);
        trajectory.outputPhasePlaneTrajectory();
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

        return(true);
    }


};


int main(int argc, char** argv) {

    ros::init(argc, argv, "totg");
    ros::NodeHandle nh;
	list<VectorXd> waypoints;
    ROS_INFO("[TOTG] TOTG starting...");

    totg planner(nh);

    ros::Rate rate(0.05);
    ROS_INFO("[TOTG] TOTG Ready!");
    while(ros::ok())
    {
        rate.sleep();
    }

	return 0;
}



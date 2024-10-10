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
        maxAcceleration_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        totg_svc = nh_.advertiseService("totg", &totg::totg_svc_cb, this);
    }


    bool totg_svc_cb(iris_support_msgs::IrisJSONsrvRequest &req, iris_support_msgs::IrisJSONsrvResponse &res)
    {
        double maxdeviation = 0.1;
        list<VectorXd> waypoints;

/*        ROS_INFO("[TOTG] TOTG request received, parsing...");
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
*/

        // Read waypoints from CSV
        std::string package_path = ros::package::getPath("trajectories-totg")+"/data/waypoints.csv";
        ifstream waypointFile(package_path.c_str());
        string line;
        while (getline(waypointFile, line)) {
            vector<string> values = split(line, ',');
            if (values.size() >= 3) {
                VectorXd waypoint(6);
                waypoint << stod(values[0]), stod(values[1]), stod(values[2]), stod(values[3]), stod(values[4]), stod(values[5]);
                waypoints.push_back(waypoint);
            }
        }
        waypointFile.close();

        ROS_INFO("wps from file %s: ", package_path.c_str());
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




#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
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


vector<string> split(const string &s, char delim) {
    vector<string> result;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

volatile bool go=false, done = false;
iris_support_msgs::IrisJSONsrvRequest svc_req;

bool totg_svc_cb(iris_support_msgs::IrisJSONsrvRequest &req, iris_support_msgs::IrisJSONsrvResponse &res)
{
    svc_req = req;
    go=true;

    // while(!done && ros::ok());

    return(true);
}

int main(int argc, char** argv) {
	list<VectorXd> waypoints;
    
    ros::init(argc, argv, "totg");
    ros::NodeHandle nh;
    ros::ServiceServer totg_svc = nh.advertiseService("totg", totg_svc_cb);

	// Read waypoints from CSV
    // std::string package_path = ros::package::getPath("trajectories-totg")+"/data/waypoints.csv";
    // ifstream waypointFile(package_path);
    // string line;
    // while (getline(waypointFile, line)) {
    //     vector<string> values = split(line, ',');
    //     if (values.size() >= 3) {
    //         VectorXd waypoint(6);
    //         waypoint << stod(values[0]), stod(values[1]), stod(values[2]), stod(values[3]), stod(values[4]), stod(values[5]);
    //         waypoints.push_back(waypoint);
    //     }
    // }
    // waypointFile.close();

    // ROS_INFO("wps from file %s: ", package_path.c_str());
    // for(VectorXd el:waypoints)
    // {
    //     std::cout << "\n" << el << std::endl;
    // }

    // define limits
	VectorXd maxAcceleration(6);
	maxAcceleration << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
	VectorXd maxVelocity(6);
	maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    ROS_INFO("waiting for svc call...");
    while(!go && ros::ok()) ros::spinOnce();
    if(!ros::ok()) exit(0);




    ROS_INFO("[TOTG] TOTG request received, parsing...");
    nlohmann::json input = nlohmann::json::parse(svc_req.json_str);
    
    int n_points = input["points"].size();
    for(int i=0; i<n_points; i++)
    {
        std::vector<double> inpoint = input["points"][i];
        Map<VectorXd> point(inpoint.data(), 6);

        waypoints.push_back(point);

    }
    ROS_INFO("wps from svc");
    for(VectorXd el:waypoints)
    {
        std::cout << "\n" << el << std::endl;
    }




    ROS_INFO("PLANNING.");

	// do the work.
    Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();

    ROS_INFO("PLAN COMPLETE.");






    // check output
	if(trajectory.isValid()) {
		double duration = trajectory.getDuration();
		cout << "Trajectory duration: " << duration << " s" << endl << endl;



		// Open output CSV file
        ofstream outputFile("trajectory_results.csv");
        outputFile << "Time,Position_1,Position_2,Position_3,Position_4,Position_5,Position_6,Velocity_1,Velocity_2,Velocity_3,Velocity_4,Velocity_5,Velocity_6" << endl;

        for(double t = 0.0; t <= duration; t += 0.1) {
            VectorXd position = trajectory.getPosition(t);
            VectorXd velocity = trajectory.getVelocity(t);

            // Write to console
            printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n",
                   t, position[0], position[1], position[2],
                   velocity[0], velocity[1], velocity[2]);

            // Write to CSV
            outputFile << t << ","
                       << position[0] << "," << position[1] << "," << position[2] << ","
                       << velocity[0] << "," << velocity[1] << "," << velocity[2] << endl;
		}
		outputFile.close();
        cout << "Results saved to trajectory_results.csv" << endl;
	}
	else {
		cout << "Trajectory generation failed." << endl;
	}
	return 0;
}




#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <Eigen/Core>
#include <trajectories-totg/Trajectory.h>
#include <trajectories-totg/Path.h>
#include <vector>

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

int main() {
	list<VectorXd> waypoints;

	// Read waypoints from CSV
    ifstream waypointFile("waypoints.csv");
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


    // define limits
	VectorXd maxAcceleration(6);
	maxAcceleration << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
	VectorXd maxVelocity(6);
	maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

	// do the work.
    Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();

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



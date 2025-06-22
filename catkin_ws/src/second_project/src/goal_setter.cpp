#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool readCSV(const std::string& filename, std::vector<std::tuple<double, double, double>>& goals) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open CSV file: %s", filename.c_str());
        return false;
    }

    std::string line;
    while (getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, theta_str;

        if (!getline(ss, x_str, ',') || !getline(ss, y_str, ',') || !getline(ss, theta_str, ',')) {
            continue;  // Skip malformed lines
        }

        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            double theta = std::stod(theta_str);
            goals.emplace_back(x, y, theta);
        } catch (const std::exception& e) {
            ROS_WARN("Invalid line in CSV: %s", line.c_str());
            continue;
        }
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle nh("~");  // private namespace for parameter access
    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, "");
    std::vector<std::tuple<double, double, double>> goals;
    if (!readCSV(csv_file, goals) || goals.empty()) {
        ROS_ERROR("No valid goals found. Exiting.");
        return 1;
    }

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base.");

    for (size_t i = 0; i < goals.size(); ++i) {
        double x, y, theta;
        std::tie(x, y, theta) = goals[i];

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        ROS_INFO("Sending goal %zu: x=%.2f, y=%.2f, theta=%.2f", i + 1, x, y, theta);
        ac.sendGoal(goal);

        ac.waitForResult();

        actionlib::SimpleClientGoalState state = ac.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %zu reached successfully.", i + 1);
        } else {
            ROS_WARN("Goal %zu failed with state: %s", i + 1, state.toString().c_str());
        }
    }

    ROS_INFO("All goals processed.");
    return 0;
}

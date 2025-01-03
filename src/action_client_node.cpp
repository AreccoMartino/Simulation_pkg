#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment2_rt/PositionVelocity.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>

ros::Publisher position_velocity_pub;

assignment2_rt::PositionVelocity pos_vel_msg;

// Action client for interacting with the Action Server
std::shared_ptr<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>> ac_ptr;

// Mutex for synchronizing access between threads
std::mutex ac_mutex;

// Atomic flags
std::atomic<bool> goal_reached(false);
std::atomic<bool> request_coordinates(false);
std::atomic<bool> goal_canceled(false);

// Callback for odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract position and velocity from odometry data
    pos_vel_msg.x = msg->pose.pose.position.x;
    pos_vel_msg.y = msg->pose.pose.position.y;
    pos_vel_msg.vel_x = msg->twist.twist.linear.x;
    pos_vel_msg.vel_z = msg->twist.twist.angular.z;

    // Publish custom message
    position_velocity_pub.publish(pos_vel_msg);
}

// Function to monitor goal status
void monitorGoalStatus() {
    ros::Rate rate(10); // Monitoring frequency
    while (ros::ok()) {
        ac_mutex.lock();
        if (ac_ptr) {
            actionlib::SimpleClientGoalState state = ac_ptr->getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Target reached!");
                goal_reached = true; // Indicate that the goal was reached
                break;
            } 
        }
        ac_mutex.unlock();
        rate.sleep();
    }
}

// Function to handle user input
void inputThread() {
    while (ros::ok()) {
        std::string command;
        ROS_INFO("Type 'set' to set a goal or 'cancel' to cancel:");

        std::getline(std::cin, command);  // Read the user's command

        if (command == "set" && !request_coordinates) {
            // Request target coordinates
            double x, y;
            ROS_INFO("Enter target coordinates (x y):");
            std::string coords;
            std::getline(std::cin, coords);  // Read the line with coordinates

            // Debug: Show input
            ROS_INFO("You entered: '%s'", coords.c_str());  // Show what was entered for debugging

            // Create a stringstream to parse the coordinates
            std::stringstream ss(coords);
            if (!(ss >> x >> y)) {  // Extract numbers from stringstream
                ROS_ERROR("Invalid input. Enter two numbers (x y) separated by a space.");
                continue;
            }

            // Set and send the goal
            assignment_2_2024::PlanningGoal goal;
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;

            ac_mutex.lock();
            ac_ptr->sendGoal(goal);
            ac_mutex.unlock();

            goal_reached = false;  // Reset goal status
            ROS_INFO("Goal sent to (%.2f, %.2f).");

            request_coordinates = true; // Mark that coordinates have been requested
            goal_canceled = false; // Reset the cancel flag when a new goal is set
        } else if (command == "cancel" && request_coordinates && !goal_canceled) {
            // Cancel the current goal if it hasn't already been canceled
            ac_mutex.lock();
            ac_ptr->cancelGoal();
            ac_mutex.unlock();
            ROS_INFO("Goal canceled.");
            goal_canceled = true; // Set the cancel flag to true after cancelation
            request_coordinates = false; // Need to reset so i can reinsert the
        } else if (goal_canceled) {
            // If the goal is already canceled, do nothing for 'cancel' command
            continue;
        } else {
            ROS_WARN("Unrecognized command. Type 'set' or 'cancel'.");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // publisher 
    position_velocity_pub = nh.advertise<assignment2_rt::PositionVelocity>("position_velocity", 10);

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Action client
    ac_ptr = std::make_shared<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>>("/reaching_goal", true);


    ROS_INFO("Waiting for Action server...");
    ac_ptr->waitForServer();
    ROS_INFO("Action server available.");

    // thread
    std::thread monitor_thread(monitorGoalStatus);
    std::thread input_thread(inputThread);

   
    ros::spin();


    monitor_thread.join();
    input_thread.join();

    return 0;
}


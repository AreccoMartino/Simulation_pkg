#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <Simulation_pkg/PositionVelocity.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>

ros::Publisher position_velocity_pub;

Simulation_pkg::PositionVelocity pos_vel_msg;

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
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED && !goal_reached) {
                ROS_INFO("Target reached!");
                ROS_INFO("Type 'set' to set a goal or 'cancel' to cancel:");
                goal_reached = true; // Indicate that the goal was reached
                request_coordinates = false; // Allow new goals to be set
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_WARN("Target could not be reached. Please set a new goal.");
                goal_reached = true; // Allow setting a new goal
                request_coordinates = false;
            }
        }
        ac_mutex.unlock();
        rate.sleep();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // Publisher
    position_velocity_pub = nh.advertise<Simulation_pkg::PositionVelocity>("position_velocity", 10);

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Action client
    ac_ptr = std::make_shared<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>>("/reaching_goal", true);

    ROS_INFO("Waiting for Action server...");
    ac_ptr->waitForServer();
    ROS_INFO("Action server available.");

    // Start monitor thread
    std::thread monitor_thread(monitorGoalStatus);

    // Main loop for user interaction
    while (ros::ok()) {
        if (!request_coordinates) {
            std::string command;
            ROS_INFO("Type 'set' to set a goal or 'cancel' to cancel:");
            std::cout.flush();
            std::getline(std::cin, command);

            if (command == "set") {
                // Request and send target coordinates
                double x, y;
                ROS_INFO("Enter target coordinates (x y):");
                std::string coords;
                std::getline(std::cin, coords);

                // Parse coordinates
                std::stringstream ss(coords);
                if (!(ss >> x >> y)) {
                    ROS_ERROR("Invalid input. Enter two numbers (x y) separated by a space.");
                    continue;
                }
                
                // update the msg with the new target
                pos_vel_msg.target_x = x;
                pos_vel_msg.target_y = y;

                // Create and send goal
                assignment_2_2024::PlanningGoal goal;
                goal.target_pose.header.frame_id = "map"; // Ensure a valid frame
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                goal.target_pose.pose.orientation.w = 1.0; // Default orientation

                ac_mutex.lock();
                ac_ptr->sendGoal(goal);
                ac_mutex.unlock();

                //request_coordinates = true;  // Mark that a goal has been requested
                goal_reached = false;        // Reset goal reached flag
                ROS_INFO("Goal sent to (%.2f, %.2f).", x, y);

            } else if (command == "cancel" /*&& request_coordinates*/) {
                // Cancel the current goal
                ac_mutex.lock();
                ac_ptr->cancelGoal();
                ac_mutex.unlock();
                ROS_INFO("Goal canceled.");
                request_coordinates = false; // Allow setting a new goal
            } else {
                ROS_WARN("Unrecognized command. Type 'set' or 'cancel'.");
            }
        }

        ros::spin();
    }

    monitor_thread.join();
    return 0;
    
}



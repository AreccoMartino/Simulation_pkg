/**
 * \file action_client_node.cpp
 * \brief Action Client node for interacting with a ROS action server.
 * \author Arecco Martino
 * \version 0.1
 * \date 27/02/2025
 *
 * \details
 * This node acts as a ROS action client that sends goals to an action server
 * and monitors their status. It also publishes the robot's position and velocity.
 *
 * \subsection Subscribed_Topics
 * - `/odom` (nav_msgs::Odometry): Receives odometry data from the robot.
 *
 * \subsection Published_Topics
 * - `/position_velocity` (Simulation_pkg::PositionVelocity): Publishes the robot's position and velocity.
 *
 * \subsection Action_Clients
 * - `/reaching_goal` (assignment_2_2024::PlanningAction): Sends goals to the action server.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <Simulation_pkg/PositionVelocity.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>

/// Publisher for the robot's position and velocity
ros::Publisher position_velocity_pub;

/// Custom message for position and velocity
Simulation_pkg::PositionVelocity pos_vel_msg;

/// Action client pointer for sending goals
std::shared_ptr<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>> ac_ptr;

/// Mutex for thread synchronization
std::mutex ac_mutex;

/// Atomic flag indicating whether the goal has been reached
std::atomic<bool> goal_reached(false);

/// Atomic flag to request new coordinates from the user
std::atomic<bool> request_coordinates(false);

/// Atomic flag indicating whether the goal has been canceled
std::atomic<bool> goal_canceled(false);

/**
 * \brief Callback function to process odometry data.
 * \param msg The odometry message containing position and velocity.
 *
 * \details
 * This function extracts the robot's position (x, y) and velocity (linear, angular)
 * from the odometry message and publishes it as a custom message.
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    pos_vel_msg.x = msg->pose.pose.position.x;
    pos_vel_msg.y = msg->pose.pose.position.y;
    pos_vel_msg.vel_x = msg->twist.twist.linear.x;
    pos_vel_msg.vel_z = msg->twist.twist.angular.z;

    position_velocity_pub.publish(pos_vel_msg);
}

/**
 * \brief Monitors the goal status and updates flags accordingly.
 *
 * \details
 * This function runs in a separate thread, checking the action server's state.
 * If the goal is reached, it updates the relevant flags and prompts the user for a new goal.
 */
void monitorGoalStatus() {
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    
    ros::Rate rate(10); // Monitoring frequency
    while (ros::ok()) {
        ac_mutex.lock();
        if (ac_ptr) {
            actionlib::SimpleClientGoalState state = ac_ptr->getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED && !goal_reached) {
                ROS_INFO("Target reached!");
                ROS_INFO("Type 'set' to set a goal or 'cancel' to cancel:");
                goal_reached = true;
                request_coordinates = false;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_WARN("Target could not be reached. Please set a new goal.");
                goal_reached = true;
                request_coordinates = false;
            }
        }
        ac_mutex.unlock();

        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * \brief Main function to initialize the ROS node and handle user interactions.
 * \param argc Number of arguments.
 * \param argv Argument vector.
 * \return 0 on successful execution.
 *
 * \details
 * This function initializes the ROS node, sets up the action client,
 * starts a monitoring thread, and continuously handles user input for setting/canceling goals.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // Initialize publisher
    position_velocity_pub = nh.advertise<Simulation_pkg::PositionVelocity>("position_velocity", 10);

    // Initialize action client
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
                double x, y;
                ROS_INFO("Enter target coordinates (x y):");
                std::string coords;
                std::getline(std::cin, coords);
                std::stringstream ss(coords);

                if (!(ss >> x >> y) || !std::isfinite(x) || !std::isfinite(y)) {
                    ROS_ERROR("Invalid input. Enter two finite numbers (x y).");
                    continue;
                }

                pos_vel_msg.target_x = x;
                pos_vel_msg.target_y = y;

                assignment_2_2024::PlanningGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                goal.target_pose.pose.orientation.w = 1.0;

                ac_mutex.lock();
                ac_ptr->sendGoal(goal);
                ac_mutex.unlock();

                goal_reached = false;
                ROS_INFO("Goal sent to (%.2f, %.2f).", x, y);

            } else if (command == "cancel") {
                ac_mutex.lock();
                ac_ptr->cancelGoal();
                ac_mutex.unlock();
                ROS_INFO("Goal canceled.");
                request_coordinates = false;
            } else {
                ROS_WARN("Unrecognized command. Type 'set' or 'cancel'.");
            }
        }

        ros::spinOnce();
    }

    monitor_thread.join();
    return 0;
}


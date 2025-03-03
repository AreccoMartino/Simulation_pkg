/**
 * \file target_service_node.cpp
 * \brief Target Service Node for handling target coordinates.
 * \author Arecco Martino
 * \version 0.1
 * \date 27/02/2025
 *
 * \details
 * This node provides a ROS service that returns the current target coordinates (x, y)
 * when requested. It also subscribes to a topic that receives updated target coordinates
 * and updates its internal state.
 *
 * \subsection Subscribed_Topics
 * - `/position_velocity` (Simulation_pkg::PositionVelocity): Receives updated target position and velocity.
 *
 * \subsection Published_Topics
 * - None.
 *
 * \subsection Services
 * - `/get_target` (Simulation_pkg::TargetService): A service that returns the current target coordinates.
 */

#include <ros/ros.h>
#include <Simulation_pkg/TargetService.h> // Service definition header
#include <Simulation_pkg/PositionVelocity.h>
#include <mutex>

// Mutex for thread safety
std::mutex target_mutex;

// Variables to store the latest target coordinates
float target_x = 0.0;
float target_y = 0.0;

/**
 * \brief Callback function for the `get_target` service.
 * \param req The request object, which is not used in this case.
 * \param res The response object that will hold the target coordinates.
 * \return Always returns true, indicating the service has been successfully processed.
 *
 * \details
 * This callback is invoked when a client calls the `get_target` service. It returns
 * the current target coordinates stored in the `target_x` and `target_y` variables.
 */
bool getTarget(Simulation_pkg::TargetService::Request &req,
               Simulation_pkg::TargetService::Response &res) {
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(target_mutex);

    // Set the response values to the current target coordinates
    res.x = target_x;
    res.y = target_y;

    ROS_INFO("Service called: Returning target coordinates (x: %.2f, y: %.2f)", res.x, res.y);

    return true;
}

/**
 * \brief Callback function to update the target coordinates from the action client.
 * \param msg The message containing the updated target position and velocity.
 *
 * \details
 * This callback is invoked when a new message is received on the `position_velocity`
 * topic. It updates the internal target coordinates (x, y) based on the received
 * message.
 */
void targetCallback(const Simulation_pkg::PositionVelocity::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(target_mutex);

    // Update the target coordinates
    target_x = msg->target_x;
    target_y = msg->target_y;
}

/**
 * \brief Main function to initialize the ROS node and handle service and subscription.
 * \param argc The number of arguments passed to the program.
 * \param argv The array of arguments passed to the program.
 * \return 0 if the program runs successfully.
 *
 * \details
 * This function initializes the ROS node, advertises the `get_target` service,
 * and subscribes to the `position_velocity` topic to receive updated target coordinates.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "target_service_node");
    ros::NodeHandle nh;

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("get_target", getTarget);

    // Subscribe to the topic publishing the target position
    ros::Subscriber target_sub = nh.subscribe("position_velocity", 10, targetCallback);

    ROS_INFO("Target service node is running.");

    // Spin to handle callbacks
    ros::spin();

    return 0;
}


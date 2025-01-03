#include <ros/ros.h>
#include <Simulation_pkg/TargetService.h> // Service definition header
#include <Simulation_pkg/PositionVelocity.h>
#include <mutex>

// Mutex for thread safety
std::mutex target_mutex;

// Variables to store the latest target coordinates
float target_x = 0.0;
float target_y = 0.0;

// Callback function for the service
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

// Subscriber callback to update the target coordinates from the action client
void targetCallback(const Simulation_pkg::PositionVelocity::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(target_mutex);

    // Update the target coordinates
    target_x = msg->x;
    target_y = msg->y;

    ROS_INFO("Updated target coordinates: (x: %.2f, y: %.2f)", target_x, target_y);
}

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


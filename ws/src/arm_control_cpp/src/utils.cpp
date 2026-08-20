#include "arm_control_cpp/utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}
double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

geometry_msgs::msg::PoseStamped create_pose(
    float x,
    float y,
    float z,
    float roll,
    float pitch,
    float yaw,
    const char* frame_id)
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = frame_id;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    pose.pose.orientation = tf2::toMsg(q);

    return pose;
}

geometry_msgs::msg::PoseStamped getPoseForState(RobotContext &ctx)
{
    switch (ctx.state) {
        default:
        case RobotState::Monitor:
            return create_pose(0.515, 0.283, 1.392, -1.574, 0.0, -1.518, "base_link"); // Redo later for extensibility
        case RobotState::QRScan:
        case RobotState::QRSearch:
            return create_pose(0.0, 0.025, 0.1, -2.754, 0.5, 0, "QR_SE"); // Redo later for extensibility
        case RobotState::Chute:
        case RobotState::ChuteRetreat:
            return create_pose(0.035, 0.027, 0.12, -2.5, 0.0, 0.0, "Chute_SE");
    }
}

geometry_msgs::msg::PoseStamped twistPick(
    geometry_msgs::msg::PoseStamped pose,
    double twist_angle
) 
{
    double twist_rad = -M_PI/2 - degreesToRadians(twist_angle);
    return create_pose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, twist_rad, 0, -M_PI/2, "base_link"); //Temp function
}

bool poseEqual(const geometry_msgs::msg::PoseStamped& pose_1,
               const geometry_msgs::msg::PoseStamped& pose_2)
{
    constexpr double POSITION_TOLERANCE = 0.001;   // meters
    constexpr double ORIENTATION_TOLERANCE = 0.05; // radians (~2.9°)

    // Position comparison
    const auto& a = pose_1.pose.position;
    const auto& b = pose_2.pose.position;

    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;

    double dist2 = dx * dx + dy * dy + dz * dz;

    if (dist2 > POSITION_TOLERANCE * POSITION_TOLERANCE)
    {
        return false;
    }

    // Orientation comparison
    tf2::Quaternion q1, q2;
    tf2::fromMsg(pose_1.pose.orientation, q1);
    tf2::fromMsg(pose_2.pose.orientation, q2);

    double angle = q1.angleShortestPath(q2);

    return angle < ORIENTATION_TOLERANCE;
}
#include "arm_control_cpp/utils.hpp"

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
            return create_pose(0.515, 0.283, 1.392, -1.574, -0.140, -1.518, "base_link"); // Redo later for extensibility
        case RobotState::QRScan:
            return create_pose(-0.209, 1.376, -0.050, 2.405, 0.964, 0.771); // Redo later for extensibility
    }
}

geometry_msgs::msg::PoseStamped twistPick(
    geometry_msgs::msg::PoseStamped pose
) 
{
    return pose; //Temp function
}

bool poseEqual(geometry_msgs::msg::PoseStamped pose_1, geometry_msgs::msg::PoseStamped pose_2) {
    const auto& a = pose_1.pose.position;
    const auto& b = pose_2.pose.position;

    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;

    double dist2 = dx*dx + dy*dy + dz*dz;

    if (dist2 < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}
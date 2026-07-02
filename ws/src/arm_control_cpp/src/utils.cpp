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

std::optional<geometry_msgs::msg::PoseStamped> get_pose_for_state(RobotContext &ctx)
{
    switch (ctx.state) {
        case RobotState::Monitor:
            return create_pose(0.059, 1.416, -0.524, 1.641, -1.519, 0.070, "base_link") // Redo later for extensibility
        case RobotState::QRScan:
            return create_pose(-0.209, 1.376, -0.050, 2.405, 0.964, 0.771) // Redo later for extensibility
        default:
            return std::nullopt;
    }
}

geometry_msgs::msg::PoseStamped twistPick(
    geometry_msgs::msg:PoseStamped pose
) {
    return pose; //Temp function
}
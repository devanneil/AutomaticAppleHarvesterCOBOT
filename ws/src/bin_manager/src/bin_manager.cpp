#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>


class BinManager : public rclcpp::Node
{
public:
    BinManager() : Node("bin_manager") {
        // ROS2 Parameters
        this->declare_parameter<std::vector<double>>(
            "bin_pose_init", std::vector<double>{-0.1, 0.1, 0.0});
        bin_pose = this->get_parameter("bin_pose_init").as_double_array();
        this->declare_parameter<std::string>(
            "bin_collision_model", std::string("package://robot_model/meshes/bin_assembly.stl"));
        bin_model = this->get_parameter("bin_collision_model").as_string();
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/bin_manager/joint_states", 10);
        planning_scene_diff_publisher_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        while (planning_scene_diff_publisher_->get_subscription_count() < 1)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        apply_bin();
        RCLCPP_INFO(get_logger(), "Apply bin successful!");

    }

private:
    std::string bin_model;
    std::vector<double> bin_pose;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;


    void apply_bin() {
        // Verify Mesh
        shapes::Mesh* mesh = shapes::createMeshFromResource(bin_model);
        shapes::ShapeMsg shape_msg;

        // Create ROS2 msg
        shapes::constructMsgFromShape(mesh, shape_msg);

        shape_msgs::msg::Mesh mesh_msg =
            boost::get<shape_msgs::msg::Mesh>(shape_msg);

        moveit_msgs::msg::CollisionObject obj;

        obj.id = "bin";
        obj.header.frame_id = "base_link";

        obj.meshes.push_back(mesh_msg);

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = bin_pose[0];
        pose.position.y = bin_pose[1];
        pose.position.z = bin_pose[2];

        obj.mesh_poses.push_back(pose);

        obj.operation = obj.ADD;
        
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = "Apple_Bin_4222026_1";
        attached_object.object = obj;

        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_->publish(planning_scene);

        // Update Joint state for TF
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = this->now();

        msg.name = {
            "bin_x_joint",
            "bin_y_joint",
            "bin_z_joint"
        };

        msg.position = {
            bin_pose[0],
            bin_pose[1],
            bin_pose[2]
        };

        joint_state_pub_->publish(msg);

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<BinManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
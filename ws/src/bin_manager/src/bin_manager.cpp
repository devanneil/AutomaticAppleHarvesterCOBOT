#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <std_msgs/msg/bool.hpp>

#include "apple_interfaces/srv/update_bin.hpp"

class BinManager : public rclcpp::Node
{
public:
    BinManager() : Node("bin_manager") {
        // ROS2 Parameters
        this->declare_parameter<std::vector<double>>(
            "bin_pose_init", std::vector<double>{-0.15, 0.15, 0.0});
        bin_pose = this->get_parameter("bin_pose_init").as_double_array();
        this->declare_parameter<std::string>(
            "bin_collision_model", std::string("package://robot_model/meshes/bin_assembly.stl"));
        bin_model = this->get_parameter("bin_collision_model").as_string();
        planning_scene_diff_publisher_ = create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);
        while (planning_scene_diff_publisher_->get_subscription_count() < 1)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        init_bin();
        RCLCPP_INFO(get_logger(), "Apply bin successful!");

        update_bin_srv_ = this->create_service<apple_interfaces::srv::UpdateBin>(
            "update_bin",
            std::bind(&BinManager::handle_update_bin, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        tf_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz
            std::bind(&BinManager::publish_bin_tf, this));

        qr_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/qr_valid",
            rclcpp::QoS(1)
                .reliable()
                .transient_local());

        std_msgs::msg::Bool false_msg;
        false_msg.data = false;
        qr_valid_pub_->publish(false_msg);

    }

private:
    std::string bin_model;
    std::vector<double> bin_pose;
    bool bin_initialized = false;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr qr_valid_pub_;
    rclcpp::Service<apple_interfaces::srv::UpdateBin>::SharedPtr update_bin_srv_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    void init_bin() {
        RCLCPP_INFO(get_logger(),
            "Publishing bin at: %.3f %.3f %.3f",
            bin_pose[0], bin_pose[1], bin_pose[2]);
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
        attached_object.link_name = "dummy_link";
        attached_object.object = obj;

        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_->publish(planning_scene);

        bin_initialized = true;
    }

    void apply_bin() {
        if (!bin_initialized) return;

        RCLCPP_INFO(get_logger(),
            "Publishing bin at: %.3f %.3f %.3f",
            bin_pose[0], bin_pose[1], bin_pose[2]);
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = bin_pose[0];
        pose.position.y = bin_pose[1];
        pose.position.z = bin_pose[2];

        moveit_msgs::msg::CollisionObject obj;

        obj.id = "bin";
        obj.header.frame_id = "base_link";
        obj.operation = obj.MOVE;
        obj.pose = pose;

        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(obj);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_->publish(planning_scene);
    }

    void publish_bin_tf()
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "dummy_link";

        tf.transform.translation.x = bin_pose[0];
        tf.transform.translation.y = bin_pose[1];
        tf.transform.translation.z = bin_pose[2];

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);
    }
    void handle_update_bin (
        const std::shared_ptr<apple_interfaces::srv::UpdateBin::Request> request,
        std::shared_ptr<apple_interfaces::srv::UpdateBin::Response> response
    ) {
        try
        {
            for (size_t i = 0; i < 3; ++i) {
                bin_pose[i] = request->new_pose[i];
            }
            apply_bin();
            response->success = true;

            std_msgs::msg::Bool true_msg;
            true_msg.data = true;
            qr_valid_pub_->publish(true_msg);
        }
        catch (std::exception& e)
        {
            response->success = false;

            std_msgs::msg::Bool false_msg;
            false_msg.data = false;
            qr_valid_pub_->publish(false_msg);
        }
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
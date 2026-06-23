#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <limits>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "apple_interfaces/msg/apple_consensus.hpp"

struct ConsensusJob
{
    std_msgs::msg::Header header;
    cv::Point2f c1;
    cv::Point2f c2;

    #ifdef ENABLE_PIPE_TIMING
    std::chrono::steady_clock::time_point t_enqueue;
    #endif
};

class ConsensusExtractorNode : public rclcpp::Node
{
public:
    ConsensusExtractorNode() : Node("consensus_extractor") {
        // Parameters
        declare_parameter<std::string>(
            "cam_sn",
            "GDS871PBAA7110621"
        );
        CAM_SN = get_parameter("cam_sn").as_string();
        declare_parameter<std::string>(
            "arm_num",
            "arm1"
        );
        ARM_NUM = get_parameter("arm_num").as_string();
        declare_parameter<std::string>(
            "cam_frame_id",
            "arm1_cam_color_frame"
        );
        CAM_FRAME_ID = get_parameter("cam_frame_id").as_string();
        declare_parameter<std::string>(
            "depth_frame_id",
            "arm1_cam_transformedDepth_frame"
        );
        DEPTH_FRAME_ID = get_parameter("depth_frame_id").as_string();
        declare_parameter<int>(
            "num_workers",
            8
        );
        NUM_WORKERS = get_parameter("num_workers").as_int();
        declare_parameter<int>(
            "ray_steps",
            30
        );
        RAYSTEPS = get_parameter("ray_steps").as_int();
        declare_parameter<float>(
            "ray_near_limit",
            0.5
        );
        RAYNEARLIMIT = get_parameter("ray_near_limit").as_double();
        declare_parameter<float>(
            "ray_far_limit",
            2.0
        );
        RAYFARLIMIT = get_parameter("ray_far_limit").as_double();
        declare_parameter<std::vector<double>>(
            "wall_normal",
            {1.0, 0.0, 0.0});

        declare_parameter<std::vector<double>>(
            "outward_normal",
            {0.0, -1.0, 0.0});
        auto wall = get_parameter("wall_normal").as_double_array();
        auto outward = get_parameter("outward_normal").as_double_array();
        wall_normal = Eigen::Vector3d(wall[0], wall[1], wall[2]);
        outward_normal = Eigen::Vector3d(outward[0], outward[1], outward[2]);
        wall_normal.normalize();
        outward_normal.normalize();
        // Subscribers
        std::string cloud_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/transformedDepth/image_raw";
        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            cloud_topic,
            10,
            std::bind(&ConsensusExtractorNode::depthCallback, this, std::placeholders::_1)
        );

        std::string depth_info_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/transformedDepth/camera_info";
        depth_info_sub_= this->create_subscription<sensor_msgs::msg::CameraInfo>(
            depth_info_topic,
            10,
            std::bind(&ConsensusExtractorNode::depthInfoCallback, this, std::placeholders::_1)
        );

        std::string vis_cloud_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/depth/points";
        depth_vis_sub_= this->create_subscription<sensor_msgs::msg::PointCloud2>(
            vis_cloud_topic,
            10,
            std::bind(&ConsensusExtractorNode::publishVizCloud, this, std::placeholders::_1)
        );
        // std::string info_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/color/camera_info";
        // cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        //     info_topic,
        //     10,
        //     std::bind(&ConsensusExtractorNode::camInfoCallback, this, std::placeholders::_1)
        // );

        std::string consensus_topic = "/" + ARM_NUM + "/apple_consensus";
        consensus_sub_ = this->create_subscription<apple_interfaces::msg::AppleConsensus> (
            consensus_topic,
            10,
            std::bind(&ConsensusExtractorNode::consensusCallback, this, std::placeholders::_1)
        );

        // Publishers
        std::string ref_cloud_topic = "/" + ARM_NUM + "/point_cloud_vis";
        ref_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ref_cloud_topic,
            10
        );

        std::string apple_pose_topic = "/" + ARM_NUM + "/apple_locations";
        apple_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped> (
            apple_pose_topic,
            10
        );
        // Compressed point cloud sub
        // vis_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(2000), // .5 Hz
        //     std::bind(&ConsensusExtractorNode::publishVizCloud, this)
        // );
        // TF Interface
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ConsensusExtractorNode::checkTfReady, this)
        );

        // Create Worker Threads        
        workers_.reserve(NUM_WORKERS);

        for (int i = 0; i < NUM_WORKERS; ++i)
        {
        workers_.emplace_back(
            &ConsensusExtractorNode::workerLoop,
            this
        );
        }

        RCLCPP_INFO(
        this->get_logger(),
        "ConsensusExtractorNode initialized with %d workers",
        NUM_WORKERS
        );


        // Initialize variables
        K_ = cv::Mat::eye(3, 3, CV_32F);
        latest_depth_ = cv::Mat();
        depth_initialized = false;
        running_ = true;
    }
    ~ConsensusExtractorNode()
    {
        running_ = false;

        queue_cv_.notify_all();

        for (auto& worker : workers_)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "ConsensusExtractorNode shut down cleanly"
        );
    }

private:
    // ===== PAREMETERS ======
    std::string CAM_SN;
    std::string ARM_NUM;
    std::string CAM_FRAME_ID;
    std::string DEPTH_FRAME_ID;
    int NUM_WORKERS;
    int RAYSTEPS;
    float RAYNEARLIMIT;
    float RAYFARLIMIT;
    Eigen::Vector3d wall_normal;
    Eigen::Vector3d outward_normal;
    // ===== ROS =====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    //rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<apple_interfaces::msg::AppleConsensus>::SharedPtr consensus_sub_; // replace with your msg
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_vis_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr apple_pose_pub_;

    rclcpp::TimerBase::SharedPtr vis_timer_;
    // ===== TF =====
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    bool tf_ready_ = false;

    // ===== Camera intrinsics =====
    cv::Mat K_; // 3x3 intrinsic matrix
    cv::Mat D_; // Camera Distortion Model
    bool camera_initialized = false;
    int image_width_;
    int image_height_;
    Eigen::Isometry3f T_depth_rgb_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;

    // ===== Point cloud state =====
    cv::Mat latest_depth_;
    std::mutex depth_mutex_;
    bool depth_initialized;

    // ===== Job queue =====
    std::queue<ConsensusJob> job_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    // ===== Worker threads =====
    std::vector<std::thread> workers_;
    std::atomic<bool> running_ = true;

private:
    // callbacks
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    //void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void consensusCallback(const apple_interfaces::msg::AppleConsensus::SharedPtr msg); // replace type
    void checkTfReady();
    void publishVizCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // workers
    void workerLoop();

    // processing
    void processJob(const ConsensusJob& job);

    // geometry
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractROI(
        const std::vector<cv::Point2f>& corners,
        const cv::Mat& cloud);

    // utils
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLatestCloud();
};
void ConsensusExtractorNode::depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(depth_mutex_);

    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    latest_depth_ = cv_ptr->image.clone();
    depth_initialized = true;
}
void ConsensusExtractorNode::depthInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_initialized) return;
    if (!tf_ready_) return;

    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    K_ = (cv::Mat_<double>(3,3) <<
        fx_, 0,   cx_,
        0,   fy_, cy_,
        0,   0,   1);

    D_ = cv::Mat(msg->d, true);  // copy distortion vector

    image_width_ = msg->width;
    image_height_ = msg->height;

    camera_initialized = true;

    RCLCPP_INFO(get_logger(), "Camera initialized!");

    RCLCPP_INFO(get_logger(),
        "CameraInfo:"
        "\n  width=%u"
        "\n  height=%u"
        "\n  distortion_model=%s"
        "\n  fx[0]=%f cx[2]=%f fy[4]=%f cy[5]=%f",
        msg->width,
        msg->height,
        msg->distortion_model.c_str(),
        msg->k[0],
        msg->k[2],
        msg->k[4],
        msg->k[5]);

    depth_info_sub_.reset();
}
void ConsensusExtractorNode::consensusCallback(const apple_interfaces::msg::AppleConsensus::SharedPtr msg) {
    uint32_t u1 = msg->u1;
    uint32_t u2 = msg->u2;
    uint32_t v1 = msg->v1;
    uint32_t v2 = msg->v2;
    
    ConsensusJob job;
    job.header = msg->header;
    job.c1 = cv::Point2d(u1, v1);
    job.c2 = cv::Point2d(u2, v2);

    #ifdef ENABLE_PIPE_TIMING
    job.t_enqueue = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Job created!");
    #endif

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        job_queue_.push(job);
    }

    queue_cv_.notify_one();
} 
void ConsensusExtractorNode::checkTfReady()
{
    if (tf_ready_)
        return;

    try
    {
        if (!tf_buffer_->canTransform(
                CAM_FRAME_ID,
                DEPTH_FRAME_ID,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5)))
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for TF: %s -> %s",
                DEPTH_FRAME_ID.c_str(),
                CAM_FRAME_ID.c_str()
            );
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg =
            tf_buffer_->lookupTransform(
                CAM_FRAME_ID,
                DEPTH_FRAME_ID,
                tf2::TimePointZero
            );

        T_depth_rgb_ = tf2::transformToEigen(tf_msg).cast<float>();

        tf_ready_ = true;

        RCLCPP_INFO(this->get_logger(), "TF ready: camera transform initialized");

        // Optional: stop timer once ready
        tf_timer_->cancel();
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "TF error: %s", ex.what()
        );
    }
}
void ConsensusExtractorNode::publishVizCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(pcl_cloud, *cloud);

    if (cloud->empty()) {
        return;
    };
    // Optional: voxel downsample (HIGHLY recommended for RViz)
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f); // tune for speed vs detail
    voxel.filter(*filtered);
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*filtered, out_msg);

    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = msg->header.frame_id;

    ref_point_cloud_pub_->publish(out_msg);
}
void ConsensusExtractorNode::workerLoop()
{
    while (rclcpp::ok() && running_)
    {
        ConsensusJob job;

        // Wait for work
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            queue_cv_.wait(
                lock,
                [this]()
                {
                    return !job_queue_.empty() || !running_;
                }
            );

            // Shutdown condition
            if (!running_)
            {
                return;
            }

            // Get next job
            job = job_queue_.front();
            job_queue_.pop();
        }

        // Ensure camera is initialized
        if (!camera_initialized)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Camera info not initialized yet"
            );

            continue;
        }

        try
        {
            // Process the consensus job
            processJob(job);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Worker exception: %s",
                e.what()
            );
        }
    }
}

// processing
void ConsensusExtractorNode::processJob(const ConsensusJob& job)
{
    cv::Mat depth;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth = latest_depth_.clone();
    }

    if (depth.empty()) return;

    std::vector<cv::Point2f> corners = {job.c1, job.c2};

    auto roi = extractROI(corners, depth);

    if (roi->empty()) return;

    Eigen::Vector3d centroid(0,0,0);

    for (const auto& p : roi->points)
        centroid += Eigen::Vector3d(p.x, p.y, p.z);

    centroid /= roi->size();

    // Lookup transform from source to target frame
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
            "base_link",               // target frame
            CAM_FRAME_ID, // source frame
            tf2::TimePointZero   // latest available
        );
    Eigen::Isometry3d T_depth_base = tf2::transformToEigen(transform_stamped);
    Eigen::Vector3d point_d =  T_depth_base * centroid;

    // Orientation Calculation
    Eigen::Vector3d z = wall_normal;
    Eigen::Vector3d x = outward_normal;

    // Make them exactly perpendicular
    x = (x - x.dot(z) * z).normalized();

    Eigen::Vector3d y = z.cross(x).normalized();

    // Recompute to guarantee orthogonality
    x = y.cross(z).normalized();

    Eigen::Matrix3d R;

    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    // Create Pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = this->now();

    pose.pose.position.x = point_d.x();
    pose.pose.position.y = point_d.y();
    pose.pose.position.z = point_d.z() + 0.03;

    Eigen::Quaterniond q(R);
    q.normalize();

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    apple_pose_pub_->publish(pose);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConsensusExtractorNode::extractROI(
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& depth)
{
    auto roi = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (depth.empty()) return roi;

    int min_u = std::numeric_limits<int>::max();
    int min_v = std::numeric_limits<int>::max();
    int max_u = 0;
    int max_v = 0;

    for (const auto& c : corners)
    {
        min_u = std::min(min_u, (int)c.x);
        min_v = std::min(min_v, (int)c.y);
        max_u = std::max(max_u, (int)c.x);
        max_v = std::max(max_v, (int)c.y);
    }

    for (int v = min_v; v <= max_v; v++)
    {
        for (int u = min_u; u <= max_u; u++)
        {
            uint16_t d = depth.at<uint16_t>(v, u);

            if (d == 0 || d == 65535) continue;

            float Z = d * 0.001f; // mm → meters

            pcl::PointXYZ p;

            p.z = Z;
            p.x = (u - cx_) * Z / fx_;
            p.y = (v - cy_) * Z / fy_;

            roi->push_back(p);
        }
    }

    return roi;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ConsensusExtractorNode>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
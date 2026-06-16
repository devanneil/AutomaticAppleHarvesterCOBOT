#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "apple_interfaces/msg/apple_consensus.hpp"

const std::string CAM_SN = "GDS871PBAA7110621";
const std::string ARM_NUM = "arm1";
const std::string CAM_FRAME_ID = "arm1_cam_color_frame";
const std::string DEPTH_FRAME_ID = "arm1_cam_points_frame";
const int NUM_WORKERS = 8;
const int RAYSTEPS = 20;
const float RAYNEARLIMIT = 1.0;
const float RAYFARLIMIT = 2.0;
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
        // Subscribers
        std::string cloud_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/depth/points";
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            10,
            std::bind(&ConsensusExtractorNode::cloudCallback, this, std::placeholders::_1)
        );

        std::string info_topic = "/" + ARM_NUM + "_cam/" + CAM_SN + "/color/camera_info";
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic,
            10,
            std::bind(&ConsensusExtractorNode::camInfoCallback, this, std::placeholders::_1)
        );

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
        latest_cloud_ = nullptr;
        running_ = true;

        cloud_buffer_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        latest_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_buffer_.reset(new pcl::PointCloud<pcl::PointXYZ>());
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
    // ===== ROS =====
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<apple_interfaces::msg::AppleConsensus>::SharedPtr consensus_sub_; // replace with your msg

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr apple_pose_pub_;

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
    Eigen::Isometry3f T_rgb_depth_;

    // ===== Point cloud state =====
    std::mutex cloud_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buffer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;

    // ===== Job queue =====
    std::queue<ConsensusJob> job_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    // ===== Worker threads =====
    std::vector<std::thread> workers_;
    std::atomic<bool> running_ = true;

private:
    // callbacks
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void consensusCallback(const apple_interfaces::msg::AppleConsensus::SharedPtr msg); // replace type
    void checkTfReady();

    // workers
    void workerLoop();

    // processing
    void processJob(const ConsensusJob& job, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg);

    // geometry
    std::vector<Eigen::Vector3f> cornerToRay(const cv::Point2f& c1, const cv::Point2f& c2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractROI(
        const std::vector<Eigen::Vector3f>& cornersRay,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // utils
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLatestCloud();
};
void ConsensusExtractorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    cloud_buffer_->clear();
    pcl::fromROSMsg(*msg, *cloud_buffer_);

    std::swap(cloud_buffer_, latest_cloud_);
}
void ConsensusExtractorNode::camInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_initialized) return;
    if (!tf_ready_) return;

    float fx_ = msg->k[0];
    float fy_ = msg->k[4];
    float cx_ = msg->k[2];
    float cy_ = msg->k[5];

    K_ = (cv::Mat_<double>(3,3) <<
        fx_, 0,   cx_,
        0,   fy_, cy_,
        0,   0,   1);

    D_ = cv::Mat(msg->d, true);  // copy distortion vector

    image_width_ = msg->width;
    image_height_ = msg->height;

    camera_initialized = true;

    RCLCPP_INFO(get_logger(), "Camera initialized!");

    cam_info_sub_.reset();
}
void ConsensusExtractorNode::consensusCallback(const apple_interfaces::msg::AppleConsensus::SharedPtr msg) {
    uint8_t u1 = msg->u1;
    uint8_t u2 = msg->u2;
    uint8_t v1 = msg->v1;
    uint8_t v2 = msg->v2;
    
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

        T_rgb_depth_ = tf2::transformToEigen(tf_msg).cast<float>();

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

        // Ensure cloud exists
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg;

        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);

            if (!latest_cloud_)
            {
                continue;
            }

            cloud_msg = latest_cloud_;
        }

        try
        {
            // Process the consensus job
            processJob(job, cloud_msg);
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
void ConsensusExtractorNode::processJob(const ConsensusJob& job, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg)
{   
    #ifdef ENABLE_PIPE_TIMING
    auto t_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Worker Consumed Job!");
    #endif
    // Extract region of interest
    std::vector<Eigen::Vector3f> rectangle = cornerToRay(job.c1, job.c2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud = extractROI(rectangle, cloud_msg);
    // Calculate centroid
    Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*region_cloud, centroid);
    // Translate to base_link
    geometry_msgs::msg::TransformStamped transform_stamped =
    tf_buffer_->lookupTransform(
        DEPTH_FRAME_ID,  // Target frame
        "base_link",    // Source frame
        tf2::TimePointZero  // Latest available
    );
    Eigen::Vector3d centroid3(centroid.x(), centroid.y(), centroid.z());
    Eigen::Isometry3d T_depth_base = tf2::transformToEigen(transform_stamped);
    Eigen::Vector3d point_d = T_depth_base * centroid3;
    Eigen::Vector3f point(point_d.x(), point_d.y(), point_d.z());
    // Create Pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "base_link";
    // Position
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    // Orientation Quaternion
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    // Publish Centroid
    apple_pose_pub_->publish(pose);
    #ifdef ENABLE_PIPE_TIMING
    auto t_end = std::chrono::steady_clock::now();

    auto end_to_end =
        std::chrono::duration_cast<std::chrono::microseconds>(
            t_end - job.t_enqueue
        ).count();

    auto pipeline =
        std::chrono::duration_cast<std::chrono::microseconds>(
            t_end - t_start
        ).count();

    RCLCPP_INFO(this->get_logger(),
        "Pipeline timing: total=%ld µs | internal=%ld µs",
        end_to_end,
        pipeline
    );
    #endif
}

// geometry
std::vector<Eigen::Vector3f> ConsensusExtractorNode::cornerToRay(const cv::Point2f& c1, const cv::Point2f& c2)
{
    std::vector<cv::Point2f> distortedVec = { c1, c2 };
    std::vector<cv::Point2f> dst;
    // Undistort from rgb to color frame
    cv::undistortPoints(distortedVec, dst, K_, D_, cv::noArray(), K_);

    cv::Point2f uv1 = dst[0];
    cv::Point2f uv2 = dst[1];

    // Transform from color frame to depth frame
    Eigen::Vector3f ray_rgb_1(uv1.x, uv1.y, 1.0);
    Eigen::Vector3f ray_rgb_2(uv2.x, uv2.y, 1.0);
    Eigen::Vector3f ray1 = T_rgb_depth_.linear() * ray_rgb_1;
    Eigen::Vector3f ray2 = T_rgb_depth_.linear() * ray_rgb_2;
    
    std::vector<Eigen::Vector3f> corners = { ray1, ray2};
    return corners;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConsensusExtractorNode::extractROI(
    const std::vector<Eigen::Vector3f>& cornersRay,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    )
{
    // KD Tree for optimized search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    // Variables for ray search
    int n = RAYSTEPS;
    float k = (RAYFARLIMIT - RAYNEARLIMIT) / n;

    // Origin at sensor
    Eigen::Vector3f origin(0,0,0);

    Eigen::Vector3f dir1 = cornersRay[0].normalized();
    Eigen::Vector3f dir2 = cornersRay[1].normalized();

    std::vector<pcl::PointXYZ> c1points(n);
    std::vector<pcl::PointXYZ> c2points(n);

    // Vectors to store result
    std::vector<int> indices(1);
    std::vector<float> distances(1);

    int c1_best_idx = 0;
    int c2_best_idx = 0;

    float c1_best_dist = std::numeric_limits<float>::max();
    float c2_best_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < n; ++i)
    {
        float t = RAYNEARLIMIT + i * k;

        Eigen::Vector3f p1 = origin + t * dir1;
        Eigen::Vector3f p2 = origin + t * dir2;

        c1points[i] = pcl::PointXYZ(p1.x(), p1.y(), p1.z());
        c2points[i] = pcl::PointXYZ(p2.x(), p2.y(), p2.z());

        if (kdtree.nearestKSearch(c1points[i], 1, indices, distances) > 0)
        {
            if (distances[0] < c1_best_dist)
            {
                c1_best_dist = distances[0];
                c1_best_idx = i;
            }
        }

        if (kdtree.nearestKSearch(c2points[i], 1, indices, distances) > 0)
        {
            if (distances[0] < c2_best_dist)
            {
                c2_best_dist = distances[0];
                c2_best_idx = i;
            }
        }
    }

    // Corners defining cube
    pcl::PointXYZ c1_best = c1points[c1_best_idx];
    pcl::PointXYZ c2_best = c2points[c2_best_idx];
    Eigen::Vector4f world_c1(c1_best.x, c1_best.y, c1_best.z, 1);
    Eigen::Vector4f world_c2(c2_best.x, c2_best.y, c2_best.z, 1);

    pcl::CropBox<pcl::PointXYZ> crop;
    auto roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>()
    );
    crop.setInputCloud(cloud);

    crop.setMin(world_c1);
    crop.setMax(world_c2);

    crop.filter(*roi_cloud);

    return roi_cloud;
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
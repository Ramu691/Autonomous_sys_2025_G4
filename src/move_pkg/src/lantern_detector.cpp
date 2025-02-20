#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>       // <-- for text publishing
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <sstream>
#include <std_msgs/Int16.h>

class LanternDetector {
public:
    LanternDetector() 
      : tf_buffer_(),
        tf_listener_(tf_buffer_),
        lantern_count_(0)
    {
        // ROS Subscribers
        semantic_sub_ = nh_.subscribe(
            "/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 10,
            &LanternDetector::semanticCallback, this);
        
        depth_sub_ = nh_.subscribe(
            "/realsense/depth/image", 10, 
            &LanternDetector::depthCallback, this);
        
        camera_info_sub_ = nh_.subscribe(
            "/realsense/depth/camera_info", 10,
            &LanternDetector::cameraInfoCallback, this);

        // ROS Publishers
        // 1) Numeric positions
        lantern_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/lantern_positions", 10);

        // 2) Human-readable text
        lantern_text_pub_ = nh_.advertise<std_msgs::String>(
            "/detected_lanterns", 10);
        
        num_lantern_pub_ = nh_.advertise<std_msgs::Int16>(
            "/num_lanterns", 10);

        // Adjust the distance threshold if needed
        distance_threshold_ = 220; // meters
    }

private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber semantic_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;

    // Publishers
    ros::Publisher lantern_pub_;
    ros::Publisher lantern_text_pub_,num_lantern_pub_;

    // Depth image, camera info
    cv::Mat depth_image_;
    sensor_msgs::CameraInfo camera_info_;

    // TF for transforming points from "camera" frame to "world" frame
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Track known lantern positions (in world frame)
    std::vector<geometry_msgs::Point> known_lantern_positions_;
    int lantern_count_;                // How many unique lanterns so far
    double distance_threshold_;        // to decide if a new detection is "new"

private:

    /**
     * @brief  Callback to receive the semantic image.
     *         We find connected components of nonzero pixels,
     *         convert each to 3D, and check if it's a new lantern.
     */
    void semanticCallback(const sensor_msgs::ImageConstPtr& semantic_msg) {
        // Convert to OpenCV MONO8
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(semantic_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat semantic_image = cv_ptr->image;

        // Quickly check if we have no nonzero pixels
        if (cv::countNonZero(semantic_image) == 0) {
            ROS_INFO("No lanterns detected.");
            return;
        }

        // Find connected components (each likely one lantern)
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(
                                semantic_image, labels, stats, centroids, 8, CV_32S);

        // Prepare a numeric array of newly-discovered lantern positions
        std_msgs::Float32MultiArray lantern_positions;

        // label_idx=0 is background
        for (int label_idx = 1; label_idx < num_components; ++label_idx) {
            double cx = centroids.at<double>(label_idx, 0);
            double cy = centroids.at<double>(label_idx, 1);

            geometry_msgs::Point point_3d = pixelTo3D((int)cx, (int)cy);
            geometry_msgs::PointStamped world_point = transformToWorldFrame(point_3d);

            // Check if we have seen this lantern before
            if (isNewLantern(world_point.point)) {
                // It's a new lantern
                lantern_count_++;
                std_msgs::Int16 msg;
                msg.data = lantern_count_;
                known_lantern_positions_.push_back(world_point.point);

                float x = world_point.point.x;
                float y = world_point.point.y;
                float z = world_point.point.z;

                // 1) Add numeric data to the float array
                lantern_positions.data.push_back(x);
                lantern_positions.data.push_back(y);
                lantern_positions.data.push_back(z);

                // 2) Print to console
                ROS_INFO("Lantern %d is detected at (%.2f, %.2f, %.2f)", 
                         lantern_count_, x, y, z);

                // 3) Publish a human-readable text message
                std_msgs::String text_msg;
                std::stringstream ss;
                ss << "Lantern " << lantern_count_ 
                   << " is detected position is (" 
                   << x << ", " << y << ", " << z << ")";
                text_msg.data = ss.str();
                lantern_text_pub_.publish(text_msg);
                num_lantern_pub_.publish(msg);
            }
        }

        // Publish any newly discovered lantern positions
        if (!lantern_positions.data.empty()) {
            lantern_pub_.publish(lantern_positions);
        }
    }

    /**
     * @brief Check if a 3D point in world frame is already known or not.
     */
    bool isNewLantern(const geometry_msgs::Point& candidate) {
        // Compare candidate to each known lantern position
        for (const auto& known : known_lantern_positions_) {
            double dx = known.x - candidate.x;
            double dy = known.y - candidate.y;
            double dz = known.z - candidate.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // If the new point is within distance_threshold_, 
            // consider it the same lantern
            if (dist < distance_threshold_) {
                return false;
            }
        }
        return true; // none were close enough, so it's new
    }

    /**
     * @brief  Callback to receive the depth image.
     */
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
        // Convert depth image (16UC1)
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth_image_ = cv_ptr->image;
    }

    /**
     * @brief  Callback to receive camera info (intrinsics).
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        camera_info_ = *info_msg;
    }

    /**
     * @brief  Convert pixel (x,y) to 3D point in the camera frame, using
     *         depth_image_ and camera_info_ intrinsics.
     */
    geometry_msgs::Point pixelTo3D(int x, int y) {
        geometry_msgs::Point point;

        // Check if we have valid depth and intrinsics
        if (depth_image_.empty() || camera_info_.K.empty()) {
            ROS_WARN("Depth image or camera info not available.");
            return point;
        }

        // Depth is in mm (16-bit). Convert to meters
        float depth = depth_image_.at<uint16_t>(y, x) * 0.001f;

        // Camera intrinsic parameters
        float fx = camera_info_.K[0];
        float fy = camera_info_.K[4];
        float cx = camera_info_.K[2];
        float cy = camera_info_.K[5];

        // Convert pixel to 3D camera coords
        point.x = (x - cx) * depth / fx;
        point.y = (y - cy) * depth / fy;
        point.z = depth;

        return point;
    }

    /**
     * @brief  Transform a geometry_msgs::Point from "camera" frame to "world".
     */
    geometry_msgs::PointStamped transformToWorldFrame(const geometry_msgs::Point& point) {
        geometry_msgs::PointStamped camera_point, world_point;
        camera_point.header.frame_id = "camera"; // or your actual camera frame
        camera_point.point = point;

        try {
            tf_buffer_.transform(camera_point, world_point, "world");
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform error: %s", ex.what());
            return camera_point; // fallback if transform fails
        }
        return world_point;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lantern_detector");
    LanternDetector detector;
    ros::spin();
    return 0;
}

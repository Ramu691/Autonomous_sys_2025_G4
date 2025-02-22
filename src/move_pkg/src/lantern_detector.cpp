#include <ros/ros.h>
#include <nav_msgs/Odometry.h>             // For drone position
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

class LanternDetector {
public:
    LanternDetector() 
      : tf_buffer_(), tf_listener_(tf_buffer_), lantern_count_(0)
    {
        // 1) Drone odometry: for position in "world" frame
        odom_sub_ = nh_.subscribe(
            "/current_state_est", 1,
            &LanternDetector::odomCallback, this);

        // 2) Semantic camera
        semantic_sub_ = nh_.subscribe(
            "/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 10,
            &LanternDetector::semanticCallback, this);

        // 3) Depth image
        depth_sub_ = nh_.subscribe(
            "/realsense/depth/image", 10,
            &LanternDetector::depthCallback, this);

        // 4) Camera intrinsics
        camera_info_sub_ = nh_.subscribe(
            "/realsense/depth/camera_info", 10,
            &LanternDetector::cameraInfoCallback, this);

        // Publishers
        lantern_positions_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/lantern_positions", 10);

        lantern_text_pub_ = nh_.advertise<std_msgs::String>(
            "/detected_lanterns", 10);

        lantern_count_pub_ = nh_.advertise<std_msgs::Int16>(
            "/num_lanterns", 10);

        // Distances
        distance_threshold_ = 100.0;  // how close is "the same lantern"
        drone_lantern_dist_ = 30.0;   // must be <= 10m to count
    }

private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber semantic_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;

    // Publishers
    ros::Publisher lantern_positions_pub_;
    ros::Publisher lantern_text_pub_;
    ros::Publisher lantern_count_pub_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Depth image + camera info
    cv::Mat depth_image_;
    sensor_msgs::CameraInfo camera_info_;

    // Known lantern positions + count
    std::vector<geometry_msgs::Point> known_lantern_positions_;
    int lantern_count_;

    // Distance thresholds
    double distance_threshold_;  // for "new" lantern vs known
    double drone_lantern_dist_;  // skip if > 10m from drone

    // Drone position in "world"
    geometry_msgs::Point drone_position_;

private:
    /**
     * @brief Drone odometry callback
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        // The pose is in "world" frame
        drone_position_ = odom_msg->pose.pose.position;
    }

    /**
     * @brief Semantic camera callback: 
     *        find lantern centroids, convert to 3D in camera->world, skip if depth=0, etc.
     */
    void semanticCallback(const sensor_msgs::ImageConstPtr &semantic_msg)
    {
        // Convert to CV MONO8
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(semantic_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Semantic cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat semantic_img = cv_ptr->image;
        if (cv::countNonZero(semantic_img) == 0) {
            
            return;
        }

        // Connected components => separate lanterns
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(
            semantic_img, labels, stats, centroids, 8, CV_32S);

        // Collect newly discovered lantern coords for publishing
        std_msgs::Float32MultiArray new_lanterns;

        for (int label_idx = 1; label_idx < num_components; ++label_idx) {
            double cx = centroids.at<double>(label_idx, 0);
            double cy = centroids.at<double>(label_idx, 1);

            // Pixel coords
            int px = static_cast<int>(cx);
            int py = static_cast<int>(cy);

            // Convert pixel -> camera coords
            geometry_msgs::Point cam_point = pixelTo3D(px, py);

            // If depth=0 => skip
            if (!depth_image_.empty() &&
                px >= 0 && px < depth_image_.cols &&
                py >= 0 && py < depth_image_.rows)
            {
                uint16_t depth_mm = depth_image_.at<uint16_t>(py, px);
                if (depth_mm == 0) {
                    ROS_INFO("Detected lantern pixel but depth=0 => skipping.");
                    continue;  // skip this detection
                }

                // Debug info
                ROS_INFO("Camera coords=(%.3f,%.3f,%.3f), depth=%u mm",
                         cam_point.x, cam_point.y, cam_point.z, depth_mm);
            }

            // Transform camera->world
            geometry_msgs::PointStamped world_point = transformToWorldFrame(cam_point);

            // dist from drone
            double dist_drone_lantern = distanceBetween(drone_position_, world_point.point);
            ROS_INFO("Distance between drone and Lantern is %.2f ", dist_drone_lantern);
            // If > 10m => skip
            if (dist_drone_lantern > drone_lantern_dist_) {
                continue;
            }

            // Check if brand-new
            if (isNewLantern(world_point.point)) {
                lantern_count_++;
                known_lantern_positions_.push_back(world_point.point);

                // Log
                ROS_INFO("Lantern %d detected at (%.2f,%.2f,%.2f), dist=%.2f m",
                         lantern_count_,
                         world_point.point.x,
                         world_point.point.y,
                         world_point.point.z,
                         dist_drone_lantern);

                // Publish to new_lanterns
                new_lanterns.data.push_back(world_point.point.x);
                new_lanterns.data.push_back(world_point.point.y);
                new_lanterns.data.push_back(world_point.point.z);

                // Also publish text
                std_msgs::String text_msg;
                std::stringstream ss;
                ss << "Lantern " << lantern_count_
                   << " at (" << world_point.point.x
                   << ", " << world_point.point.y
                   << ", " << world_point.point.z
                   << "), droneDist=" << dist_drone_lantern << "m";
                text_msg.data = ss.str();
                lantern_text_pub_.publish(text_msg);

                // Publish updated count
                std_msgs::Int16 count_msg;
                count_msg.data = lantern_count_;
                lantern_count_pub_.publish(count_msg);
            }
        }

        // Publish any new lantern positions
        if (!new_lanterns.data.empty()) {
            lantern_positions_pub_.publish(new_lanterns);
        }
    }

    /**
     * @brief Convert (px,py) => 3D in camera frame
     */
    geometry_msgs::Point pixelTo3D(int px, int py)
    {
        geometry_msgs::Point pt;
        if (depth_image_.empty() || camera_info_.K.empty()) {
            ROS_WARN("Depth image or camera info not ready.");
            return pt; // (0,0,0)
        }

        uint16_t depth_raw = depth_image_.at<uint16_t>(py, px); // mm
        float depth_m = depth_raw * 0.001f;

        float fx = camera_info_.K[0];
        float fy = camera_info_.K[4];
        float cx = camera_info_.K[2];
        float cy = camera_info_.K[5];

        pt.x = (px - cx) * depth_m / fx;
        pt.y = (py - cy) * depth_m / fy;
        pt.z = depth_m;
        return pt;
    }

    /**
     * @brief Transform from "camera" to "world"
     */
    geometry_msgs::PointStamped transformToWorldFrame(const geometry_msgs::Point &cam_point)
    {
        geometry_msgs::PointStamped camera_pt, world_pt;
        camera_pt.header.frame_id = "camera"; 
        camera_pt.point = cam_point;

        try {
            tf_buffer_.transform(camera_pt, world_pt, "world");
            ROS_INFO("World coords=(%.3f,%.3f,%.3f)",
                     world_pt.point.x,
                     world_pt.point.y,
                     world_pt.point.z);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("transformToWorldFrame error: %s", ex.what());
            return camera_pt; // fallback
        }
        ROS_INFO("transformToWorldFrame worked");
        return world_pt;
    }

    /**
     * @brief Depth callback
     */
    void depthCallback(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_image_ = cv_ptr->image;
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Depth cv_bridge exception: %s", e.what());
        }
    }

    /**
     * @brief Camera info callback
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
    {
        camera_info_ = *info_msg;
    }

    /**
     * @brief Check if candidate is far from all known lanterns
     */
    bool isNewLantern(const geometry_msgs::Point &candidate)
    {
        for (const auto &known : known_lantern_positions_) {
            double d = distanceBetween(known, candidate);
            if (d < distance_threshold_) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 3D distance
     */
    double distanceBetween(const geometry_msgs::Point &p1,
                           const geometry_msgs::Point &p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lantern_detector");
    LanternDetector detector;
    ros::spin();
    return 0;
}

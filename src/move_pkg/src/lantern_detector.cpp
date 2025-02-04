#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class LanternDetector {
public:
    LanternDetector() : tf_buffer_(), tf_listener_(tf_buffer_) {
        // ROS publishers and subscribers
        semantic_sub_ = nh_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 10, &LanternDetector::semanticCallback, this);
        depth_sub_ = nh_.subscribe("/realsense/depth/image", 10, &LanternDetector::depthCallback, this);
        camera_info_sub_ = nh_.subscribe("/realsense/depth/camera_info", 10, &LanternDetector::cameraInfoCallback, this);
        // publish 3D position of detected lanterns as a list
        lantern_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/lantern_positions", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber semantic_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher lantern_pub_;

    cv::Mat depth_image_;
    sensor_msgs::CameraInfo camera_info_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void semanticCallback(const sensor_msgs::ImageConstPtr& semantic_msg) {
        // Convert semantic image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(semantic_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat semantic_image = cv_ptr->image;

        // Detect non-zero pixels (indicating detected objects)
        std::vector<cv::Point> detected_pixels;
        cv::findNonZero(semantic_image, detected_pixels);

        // semantic camera only detects lanterns
        if (detected_pixels.empty()) {
            ROS_INFO("No lanterns detected.");
            return;
        }

        // Calculate 3D positions of detected lanterns
        std_msgs::Float32MultiArray lantern_positions;
        for (const auto& pixel : detected_pixels) {
            // convert pixel into 3D point in camera frame
            geometry_msgs::Point point_3d = pixelTo3D(pixel.x, pixel.y);
            // transform the point into world frame
            geometry_msgs::PointStamped world_point = transformToWorldFrame(point_3d);

            // Add to output array and store
            lantern_positions.data.push_back(world_point.point.x);
            lantern_positions.data.push_back(world_point.point.y);
            lantern_positions.data.push_back(world_point.point.z);
        }

        // Publish the positions
        lantern_pub_.publish(lantern_positions);
    }

    // get the depth of image
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
        // Convert depth image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Update depth_image -> allow pixelto3D to use it
        depth_image_ = cv_ptr->image;
    }

    // store camera intrinsic matrix
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        camera_info_ = *info_msg;
    }

    
    geometry_msgs::Point pixelTo3D(int x, int y) {
        // Ensure depth image and camera info are available
        if (depth_image_.empty() || camera_info_.K.empty()) {
            ROS_WARN("Depth image or camera info not available.");
            return geometry_msgs::Point();
        }

        // Get depth value at the pixel
        float depth = depth_image_.at<uint16_t>(y, x) * 0.001; // Convert to meters

        // Camera intrinsic parameters
        float fx = camera_info_.K[0];
        float fy = camera_info_.K[4];
        float cx = camera_info_.K[2];
        float cy = camera_info_.K[5];

        // Convert pixel to 3D coordinates in camera frame
        geometry_msgs::Point point;
        point.x = (x - cx) * depth / fx;
        point.y = (y - cy) * depth / fy;
        point.z = depth;
        return point;
    }

    geometry_msgs::PointStamped transformToWorldFrame(const geometry_msgs::Point& point) {
        geometry_msgs::PointStamped camera_point, world_point;
        camera_point.header.frame_id = "camera";
        camera_point.point = point;

        try {
            tf_buffer_.transform(camera_point, world_point, "world");
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform error: %s", ex.what());
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

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Path.h>

class FrontierNavigator
{
public:
    FrontierNavigator(){
        if (!ros::param::get("navigatior/acceptance_radius", acceptance_radius_)) ROS_FATAL("Required parameter navigatior/acceptance_radius was not found on parameter server");
        if (!ros::param::get("navigatior/publish_des_state_frequency", publish_rate_)) ROS_FATAL("Required parameter navigatior/publish_des_state_frequency was not found on parameter server");
        if (!ros::param::get("navigatior/gain", gain_)) ROS_FATAL("Required parameter navigatior/gain was not found on parameter server");

        path_sub_ = nh_.subscribe("/rrt_path", 1, &FrontierNavigator::pathCallback, this);

        pose_sub_ = nh_.subscribe("/pose_est", 1,
                                  &FrontierNavigator::poseCallback, this);
        desired_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
                          "/desired_state", 10);
        wp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_waypoint", 5);
        // Timer to publish at publish_rate_
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_),&FrontierNavigator::timerCallback,this);     
        ROS_INFO("FrontierNavigator forwarder: Subscribing /rrt_path -> /desired_state");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_, pose_sub_;
    ros::Publisher  desired_pub_, wp_pub_;
    nav_msgs::Path last_path_;
    geometry_msgs::PoseStamped last_pose_;;
    ros::Timer timer_;
    double acceptance_radius_;
    int cur_Index_;
    int publish_rate_;
    bool path_received_;
    double gain_;

    // Function to compute quaternion from two points
    geometry_msgs::Quaternion computeQuaternion(const geometry_msgs::Point& pointA, const geometry_msgs::Point& pointB) {
        double dx = pointB.x - pointA.x;
        double dy = pointB.y - pointA.y;
        double dz = pointB.z - pointA.z;

        // Compute yaw (rotation around Z-axis)
        double yaw = std::atan2(dy, dx);

        // Compute pitch (rotation around Y-axis)
        double distance_xy = std::sqrt(dx * dx + dy * dy);
        double pitch = std::atan2(dz, distance_xy);

        // Convert to quaternion using tf2
        tf2::Quaternion q;
        q.setRPY(0, pitch, yaw);  // Roll = 0 (assumption: no tilt in X-axis)

        // Convert tf2 quaternion to geometry_msgs quaternion
        geometry_msgs::Quaternion q_msg;
        q_msg = tf2::toMsg(q);
        return q_msg;
    }


    void pathCallback(const nav_msgs::Path &msg){
        path_received_ = true;
        // Start as 1 as 0 is current position
        cur_Index_ = 1;
        last_path_ = msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped& msg) {
        if(!path_received_ || cur_Index_ >= last_path_.poses.size()){
            return;
        }        
        double dist_wp = sqrt(pow(msg.pose.position.x - last_path_.poses[cur_Index_].pose.position.x, 2) + 
                        pow(msg.pose.position.y - last_path_.poses[cur_Index_].pose.position.y, 2) + 
                        pow(msg.pose.position.z - last_path_.poses[cur_Index_].pose.position.z, 2));
        if(dist_wp < acceptance_radius_){  
            cur_Index_++;
        }
        last_pose_ = msg;
    }

    double calculate_error(double desired_pose, double actual_pose){
        return gain_ * (desired_pose - actual_pose);
    }

    void timerCallback(const ros::TimerEvent&){
        if(!path_received_ || cur_Index_ >= last_path_.poses.size()){
            return;
        }

        double delta_x = calculate_error(last_path_.poses[cur_Index_].pose.position.x, last_pose_.pose.position.x);
        double delta_y = calculate_error(last_path_.poses[cur_Index_].pose.position.y, last_pose_.pose.position.y);
        double delta_z = calculate_error(last_path_.poses[cur_Index_].pose.position.z, last_pose_.pose.position.z);     
    
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_msg;
        trajectory_msg.transforms.resize(1);
        trajectory_msg.transforms[0].translation.x = delta_x + last_pose_.pose.position.x;
        trajectory_msg.transforms[0].translation.y = delta_y + last_pose_.pose.position.y;
        trajectory_msg.transforms[0].translation.z = delta_z + last_pose_.pose.position.z;
        trajectory_msg.transforms[0].rotation = computeQuaternion(last_pose_.pose.position, last_path_.poses[cur_Index_].pose.position);

        geometry_msgs::Twist velocity_msg;
        geometry_msgs::Twist acceleration_msg;
        trajectory_msg.velocities.resize(1);
        trajectory_msg.velocities[0] = velocity_msg;
        trajectory_msg.accelerations.resize(1);
        trajectory_msg.accelerations[0] = acceleration_msg;
        desired_pub_.publish(trajectory_msg);

        // Rviz
        geometry_msgs::PoseStamped wp_msg;
		wp_msg.header.frame_id = "world";
	    wp_msg.pose.position.x = trajectory_msg.transforms[0].translation.x;
        wp_msg.pose.position.y = trajectory_msg.transforms[0].translation.y;
        wp_msg.pose.position.z = trajectory_msg.transforms[0].translation.z;
        wp_msg.pose.orientation = trajectory_msg.transforms[0].rotation;
        wp_pub_.publish(wp_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_navigator");
    FrontierNavigator fn;
    ros::spin();
    return 0;
}

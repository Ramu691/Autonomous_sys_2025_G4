#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>

class HoverDrone {
    ros::NodeHandle nh_;
    ros::Subscriber pose_subscriber_;
    ros::Publisher desired_state_publisher_;

    double current_x_, current_y_, current_z_;
    double target_x_, target_y_, target_z_;
    double current_yaw_;
    double yaw_rate_;  // radians/sec
    bool position_initialized_;
    ros::Time start_time_;
    double move_duration_; // seconds to move from start to target
    double start_x_, start_y_, start_z_; // position at start of move
    double start_yaw_;

public:
    HoverDrone()
      : current_x_(0)
      , current_y_(0)
      , current_z_(0)
      , target_x_(0)
      , target_y_(0)
      , target_z_(0)
      , current_yaw_(0)
      , yaw_rate_(M_PI / 10.0)  // ~180° in 10s
      , position_initialized_(false)
      , move_duration_(10.0)    // 10 seconds to move
    {
        pose_subscriber_ = nh_.subscribe("/pose_est", 1, &HoverDrone::poseCallback, this);
        desired_state_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);

        ROS_INFO("HoverDrone initialized. Will move to (current_x_-5, current_y_-5, current_z_+1) over 10s, rotating yaw.");
    }

    void poseCallback(const geometry_msgs::PoseStamped& pose) {
        current_x_ = pose.pose.position.x;
        current_y_ = pose.pose.position.y;
        current_z_ = pose.pose.position.z;

        // If it's our first time receiving a pose, set up the target & start
        if (!position_initialized_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_z_ = current_z_;
            start_yaw_ = getYawFromPose(pose);

            // Example: move 5 meters in -x, -y, and +1 in z from current
            target_x_ = current_x_ - 5.0;
            target_y_ = current_y_ - 5.0;
            target_z_ = current_z_ + 1.0;

            // Mark time
            start_time_ = ros::Time::now();
            position_initialized_ = true;
            ROS_INFO("HoverDrone: position initialized. Starting move at (%.2f, %.2f, %.2f).",
                     start_x_, start_y_, start_z_);
        }

        // Compute and publish the desired state at each callback
        // (Alternatively, you could use a Timer with a fixed rate.)
        publishDesiredState();
    }

    double getYawFromPose(const geometry_msgs::PoseStamped &pose) const {
        // Convert quaternion to yaw
        tf2::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void publishDesiredState() {
        if (!position_initialized_) {
            ROS_WARN("HoverDrone: pose not initialized yet. Can't publish desired state.");
            return;
        }

        // How long since we started this move?
        ros::Time now = ros::Time::now();
        double t = (now - start_time_).toSec();
        if (t > move_duration_) t = move_duration_;

        // Fraction of the move completed [0..1]
        double alpha = t / move_duration_;

        // Position interpolation
        double des_x = (1.0 - alpha) * start_x_ + alpha * target_x_;
        double des_y = (1.0 - alpha) * start_y_ + alpha * target_y_;
        double des_z = (1.0 - alpha) * start_z_ + alpha * target_z_;

        // Velocity: derivative of position w.r.t. time
        // v = (target - start)/move_duration_, constant
        // if alpha < 1, else 0
        double vx = 0.0, vy = 0.0, vz = 0.0;
        if (t < move_duration_) {
            vx = (target_x_ - start_x_) / move_duration_;
            vy = (target_y_ - start_y_) / move_duration_;
            vz = (target_z_ - start_z_) / move_duration_;
        }

        // Acceleration = 0 in this simple linear example
        double ax = 0.0, ay = 0.0, az = 0.0;

        // Yaw: rotate at a fixed yaw_rate_, starting from start_yaw_
        // (You can do the same interpolation as above if you prefer.)
        double dt = t; // time since start
        double des_yaw = start_yaw_ + yaw_rate_ * dt;
        // Keep yaw in [-pi, pi]
        if (des_yaw > M_PI) des_yaw -= 2 * M_PI;
        if (des_yaw < -M_PI) des_yaw += 2 * M_PI;

        // Yaw velocity = yaw_rate_ (if not done)
        double yaw_vel = yaw_rate_; 
        if (t >= move_duration_) {
            yaw_vel = 0.0;  // stop rotating after move_duration_
        }

        // Build the message
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.velocities.resize(1);
        msg.accelerations.resize(1);

        // 1) Position
        msg.transforms[0].translation.x = des_x;
        msg.transforms[0].translation.y = des_y;
        msg.transforms[0].translation.z = des_z;

        // 2) Orientation from des_yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, des_yaw);
        msg.transforms[0].rotation.x = q.x();
        msg.transforms[0].rotation.y = q.y();
        msg.transforms[0].rotation.z = q.z();
        msg.transforms[0].rotation.w = q.w();

        // 3) Linear velocity
        msg.velocities[0].linear.x = vx;
        msg.velocities[0].linear.y = vy;
        msg.velocities[0].linear.z = vz;

        // 4) Angular velocity
        msg.velocities[0].angular.x = 0.0;
        msg.velocities[0].angular.y = 0.0;
        msg.velocities[0].angular.z = yaw_vel;

        // 5) Linear accelerations
        msg.accelerations[0].linear.x = ax;
        msg.accelerations[0].linear.y = ay;
        msg.accelerations[0].linear.z = az;

        // 6) Angular accelerations (0 for simplicity)
        msg.accelerations[0].angular.x = 0.0;
        msg.accelerations[0].angular.y = 0.0;
        msg.accelerations[0].angular.z = 0.0;

        // Publish
        desired_state_publisher_.publish(msg);

        ROS_INFO_THROTTLE(1.0, "HoverDrone: Desired-> x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, vx=%.2f, vy=%.2f, vz=%.2f",
                          des_x, des_y, des_z, des_yaw, vx, vy, vz);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_rotate_drone");
    HoverDrone drone;
    ros::spin();
    return 0;
}

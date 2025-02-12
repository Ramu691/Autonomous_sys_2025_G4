#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

class FrontierNavigator
{
public:
    FrontierNavigator()
    {
        traj_sub_ = nh_.subscribe("/trajectory", 10,
                                  &FrontierNavigator::trajCallback, this);
        pose_sub_ = nh_.subscribe("/pose_est", 1,
                                  &FrontierNavigator::poseCallback, this);
        desired_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
                          "/desired_state", 10);

        ROS_INFO("FrontierNavigator forwarder: Subscribing /trajectory -> /desired_state");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher  desired_pub_;

    void trajCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg)
    {
        desired_pub_.publish(msg);
        ROS_INFO_THROTTLE(1.0, "Forwarding point -> pos(%.2f,%.2f,%.2f)",
                          msg.transforms[0].translation.x,
                          msg.transforms[0].translation.y,
                          msg.transforms[0].translation.z);
    }

    void poseCallback(const geometry_msgs::PoseStamped& msg) {
        // Optionally do something with the drone pose
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_navigator");
    FrontierNavigator fn;
    ros::spin();
    return 0;
}
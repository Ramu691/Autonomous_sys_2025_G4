#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Position{
    public:
        double x_position, y_position, z_position, heading;
        Position(double x_val, double y_val, double z_val, double yaw_val){
            x_position = x_val;
            y_position = y_val;
            z_position = z_val;
            heading = yaw_val;
        }
};

class TakeoffNode {
    ros::NodeHandle nh;
    ros::Subscriber state_subscriber;
    ros::Publisher des_state_publisher;
    std::string state_machine_state;
    ros::Timer timer;

    Position takeoffWaypoint = Position(-38.0, 10.0, 12.0, 3.14);
    
    public:
        TakeoffNode() {
            state_subscriber = nh.subscribe("/stm_mode", 1, &TakeoffNode::onStateStm, this);
            des_state_publisher = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
            timer = nh.createTimer(ros::Rate(10), &TakeoffNode::controlLoop, this);
        }

        void onStateStm(const std_msgs::String& cur_state) {
        	state_machine_state = cur_state.data;
            if (state_machine_state == "NAVIGATE") {
                // Job done
                ros::shutdown();
            }
        }
                
        void controlLoop(const ros::TimerEvent& t){
            
            if (state_machine_state != "TAKEOFF"){
                // Don't do anyhting if state is not takeoff
                return;
            }

            trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_msg;
            trajectory_msg.transforms.resize(1);
            trajectory_msg.transforms[0].translation.x = takeoffWaypoint.x_position;
            trajectory_msg.transforms[0].translation.y = takeoffWaypoint.z_position;
            trajectory_msg.transforms[0].translation.z = takeoffWaypoint.y_position;
            
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, takeoffWaypoint.heading);  
            trajectory_msg.transforms[0].rotation.x = quaternion.getX();
            trajectory_msg.transforms[0].rotation.y = quaternion.getY();
            trajectory_msg.transforms[0].rotation.z = quaternion.getZ();
            trajectory_msg.transforms[0].rotation.w = quaternion.getW();
            
            geometry_msgs::Twist velocity_msg;
            geometry_msgs::Twist acceleration_msg;
            trajectory_msg.velocities.resize(1);
            trajectory_msg.velocities[0] = velocity_msg;
            trajectory_msg.accelerations.resize(1);
            trajectory_msg.accelerations[0] = acceleration_msg;

            des_state_publisher.publish(trajectory_msg);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "takeoff_node");  
    TakeoffNode takeoff;
    ros::spin();
    return 0;
}

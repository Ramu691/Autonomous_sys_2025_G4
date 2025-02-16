#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define NUM_WP 2

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

class UAVController{
    ros::NodeHandle nh;
    ros::Subscriber state_subscriber,position_subscriber;
    ros::Publisher des_state_publisher, goal_publisher;
    double current_x, current_y, current_z;
    double initial_x, initial_y, initial_z;
    double delta_x, delta_y, delta_z;
    double acceptance_radius, gain;
    std::string statemachine_state;
    
    Position waypoints[NUM_WP] = {Position(-38.0, 10.0, 20.0, 3.14),Position(-324.0, 10.0, 16.0, 3.14)};  

    int waypoint_index=-1;
    double time_start = -1;
    
    public:
        UAVController(){
            state_subscriber = nh.subscribe("/stm_mode", 1, &UAVController::onStateStm, this);
            position_subscriber = nh.subscribe("/pose_est", 1, &UAVController::stateCallback, this);
            des_state_publisher = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
            if (!ros::param::get("controller/acceptance_radius", acceptance_radius)) ROS_FATAL("Required parameter controller/acceptance_radius was not found on parameter server");
            if (!ros::param::get("controller/gain", gain)) ROS_FATAL("Required parameter controller/gain was not found on parameter server");
        }

        void onStateStm(const std_msgs::String& cur_state){
        	statemachine_state = cur_state.data;
            // Reset waypoint selection based on state
            if (statemachine_state == "TAKEOFF") {
                waypoint_index = 0;
            } else if (statemachine_state == "NAVIGATE") {
                waypoint_index = 1;
            }
        }
        
        double calculate_error(double desired_pose, double actual_pose){
            return gain * (desired_pose - actual_pose);
        }
                
        void stateCallback(const geometry_msgs::PoseStamped& current_pose){   
            // Set time_start as soon as first stateCallback arrives 
            //if(statemachine_state =="NAVIGATIOacceptance_radiusN"){  
            if(time_start == -1 ){
                time_start = ros::Time::now().toSec();
            }
            current_x = current_pose.pose.position.x;
            current_y = current_pose.pose.position.y;
            current_z = current_pose.pose.position.z;
            
            //  // Check if the UAV has reached the current waypoint
            // double distance_to_waypoint=sqrt(pow(current_x - waypoints[waypoint_index].x_position, 2) + 
            //         pow(current_y - waypoints[waypoint_index].y_position, 2) + 
            //         pow(current_z - waypoints[waypoint_index].z_position, 2));
            

            initial_x = current_x;
            initial_y = current_y;
            initial_z = current_z;
            
            delta_x = calculate_error(waypoints[waypoint_index].x_position, current_x);
            delta_y = calculate_error(waypoints[waypoint_index].y_position, current_y);
            delta_z = calculate_error(waypoints[waypoint_index].z_position, current_z);
            
            if(waypoint_index < NUM_WP){
                double time_elapsed = ros::Time::now().toSec() - time_start;        
            
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_msg;
                trajectory_msg.transforms.resize(1);
                trajectory_msg.transforms[0].translation.x = delta_x * time_elapsed + initial_x;
                trajectory_msg.transforms[0].translation.y = delta_y * time_elapsed + initial_y;
                trajectory_msg.transforms[0].translation.z = delta_z * time_elapsed + initial_z;
                
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, waypoints[waypoint_index].heading);  
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
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "nav_to_cave_entrance");  
    UAVController uav;
    ros::spin();
    
    return 0;
}

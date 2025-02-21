#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
// Include the custom frontier message
#include <custom_msgs/FrontierGoalMsg.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h> 

#include <custom_msgs/FrontierGoalMsg.h>




// Define mission states.
enum RobotState { IDLE, TAKEOFF, NAVIGATE, EXPLORE, LAND };

// Global mission state and current feedback.
RobotState current_state = IDLE;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Point latest_frontier_point;

std_msgs::Int16 num_of_lantern;
// Counter to track consecutive condition validations
int frontier_check_counter = 0;
const int REQUIRED_COUNT = 20;  // Number of consecutive times condition must be true

// Callback to update current state from /current_state_est.
void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose.header = msg->header;
  current_pose.pose = msg->pose.pose;
}

// --- Frontier Goal Handling ---
// Global variable for latest frontier goal.
custom_msgs::FrontierGoalMsg latest_frontier_goal;
bool frontier_goal_received = false;

// Callback for /frontier_goal.
void frontierGoalCallback(const custom_msgs::FrontierGoalMsg::ConstPtr& msg) {
  latest_frontier_goal = *msg;
  frontier_goal_received = true;
  latest_frontier_point.x = msg->point.x;
  latest_frontier_point.y = msg->point.y;
  latest_frontier_point.z = msg->point.z;
  // ROS_INFO("Frontier goal received: x=%f, y=%f, z=%f",
  //         msg->point.x, msg->point.y, msg->point.z);
}

// void num_lantern_callback(const std_msgs::String::ConstPtr& num) {
//     try {
//         num_of_lantern = std::stoi(num->data);
//     } catch (const std::exception& e) {
//         ROS_ERROR("Failed to convert num_lantern string to int: %s", e.what());
//     }
// }
void num_lantern_callback(const std_msgs::Int16::ConstPtr& msg) {
        num_of_lantern.data = msg->data;  
        //ROS_INFO("Received number of lanterns: %d", num_of_lantern.data);       
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh;
  num_of_lantern.data=0;
  
  ros::Publisher desired_state_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 1, true);
  ros::Publisher stm_mode_pub = nh.advertise<std_msgs::String>("/stm_mode", 10);

  // Subscribers.
  ros::Subscriber current_state_sub = nh.subscribe("/current_state_est", 10, currentStateCallback);
  // Subscribe to frontier goal from the frontier detector.
  ros::Subscriber frontier_goal_sub = nh.subscribe("/frontier_goal", 1, frontierGoalCallback);
  ros::Subscriber lantern_goal_sub = nh.subscribe("/num_lanterns", 1, num_lantern_callback);

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);

  // Define waypoints.
  geometry_msgs::PoseStamped cave_entrance_goal;
  cave_entrance_goal.pose.position.x = -321.0;
  cave_entrance_goal.pose.position.y = 10.0;
  cave_entrance_goal.pose.position.z = 15.0;
  cave_entrance_goal.pose.orientation.w = 3.14;
  
  geometry_msgs::PoseStamped take_off;
  take_off.pose.position.x = -38;
  take_off.pose.position.y = 10;
  take_off.pose.position.z = 20;
  take_off.pose.orientation.w = 1.0;

  ros::Rate rate(5);
  ros::Time last_transition_time = ros::Time::now();
  ros::Time state_entry_time = ros::Time::now();
  ros::Time last_reset_time;  // Time tracking for counter reset
  const double RESET_INTERVAL = 50.0;  // Reset every 5 seconds

  // Transition parameters.
  double takeoff_altitude = 15.0;
  double takeoff_threshold = 2.0;
  double frontier_threshold =1.3;
  double cave_threshold = 5.0;  // Increased threshold so the state transitions when within 5 m of the cave entrance.
  double landing_threshold = 0.5;
  double explore_duration = 100000.0; // For now, exploration runs indefinitely (or until manually terminated).

  
//   void num_lantern_callback(const std_msgs::Int16::ConstPtr& num) {
//     try {
//         num_of_lantern = std::stoi(num->data);
//     } catch (const std::exception& e) {
//         ROS_ERROR("Failed to convert num_lantern to int: %s", e.what());
//     }
// }

  while (ros::ok()) {
    if ((ros::Time::now() - last_transition_time).toSec() >= 3.0) {
      std_msgs::String mode_msg;
      switch (current_state) {
        case IDLE: {
            ROS_INFO("State: IDLE");
            mode_msg.data = "IDLE";
            stm_mode_pub.publish(mode_msg);          
                
                ROS_INFO("Transitioning from IDLE to TAKEOFF");
                current_state=TAKEOFF;
                ROS_INFO("State: TAKEOFF");
                                      
          } break;

        case TAKEOFF: {
            //ROS_INFO("State: TAKEOFF");
            mode_msg.data = "TAKEOFF";
            stm_mode_pub.publish(mode_msg);
            if(sqrt(pow(current_pose.pose.position.x - take_off.pose.position.x, 2) + 
                    pow(current_pose.pose.position.y - take_off.pose.position.y, 2) + 
                    pow(current_pose.pose.position.z - take_off.pose.position.z, 2)) < takeoff_threshold ) {
              ROS_INFO("Takeoff complete");
              current_state = NAVIGATE;
              ROS_INFO("State: NAVIGATE");
              //state_entry_time = ros::Time::now();
            }
          } break;

        case NAVIGATE: {
            //ROS_INFO("State: NAVIGATE");
            mode_msg.data = "NAVIGATE";
            stm_mode_pub.publish(mode_msg);
            double dx = current_pose.pose.position.x - cave_entrance_goal.pose.position.x;
            double dy = current_pose.pose.position.y - cave_entrance_goal.pose.position.y;
            double dz = current_pose.pose.position.z - cave_entrance_goal.pose.position.z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            ROS_INFO("Distance to cave entrance: %f", dist);
            if (dist < cave_threshold) {
              ROS_INFO("Cave entrance reached.");
              current_state = EXPLORE;
              ROS_INFO("State: EXPLORE");
              
            }
          } break;

        case EXPLORE: {
            //ROS_INFO("State: EXPLORE");
            mode_msg.data = "EXPLORE";
            stm_mode_pub.publish(mode_msg);
            
            // //   ROS_INFO("Landing.");
            // //   current_state = LAND;
            // }
            // //else if(!frontier_goal_received && tot_num_lanters_detected==5){
            // if (!frontier_goal_received && num_of_lantern.data == 5) {
            //ros::Time current_time = ros::Time::now();  // Get the current time
            // if ((current_time - last_reset_time).toSec() >= RESET_INTERVAL) {
            //     frontier_check_counter = 0;
            //     last_reset_time = current_time;  // Update reset time
            //     ROS_INFO("Counter reset after 5 seconds.");
            // }
            if(sqrt(pow(current_pose.pose.position.x - latest_frontier_point.x, 2) + 
                    pow(current_pose.pose.position.y - latest_frontier_point.y, 2) + 
                    pow(current_pose.pose.position.z - latest_frontier_point.z, 2)) <= frontier_threshold ){
                    frontier_check_counter++;  // Increment counter

                    if (frontier_check_counter >= REQUIRED_COUNT) {
                          frontier_goal_received = false;
                          ROS_INFO("Frontier goal disabled after 10 consecutive checks.");
                      }
                      
                    }
            
            // if (!frontier_goal_received && num_of_lantern.data == 5 ) {
            if (!frontier_goal_received && num_of_lantern.data == 5 ) {
                ROS_INFO("Landing.");
                current_state = LAND;
                ROS_INFO("State: LAND");
            }

          } break;
        case LAND: {
            ROS_INFO("State: LAND");
            mode_msg.data = "LAND";
            stm_mode_pub.publish(mode_msg);
            trajectory_msgs::MultiDOFJointTrajectoryPoint land_traj;
            land_traj.transforms.resize(1);
            land_traj.velocities.resize(1);
            land_traj.accelerations.resize(1);
            land_traj.transforms[0].translation.x = current_pose.pose.position.x;
            land_traj.transforms[0].translation.y = current_pose.pose.position.y;
            land_traj.transforms[0].translation.z = current_pose.pose.position.z;

            land_traj.transforms[0].translation.z = land_traj.transforms[0].translation.z;

            land_traj.transforms[0].rotation.x = 0;
            land_traj.transforms[0].rotation.y = 0;
            land_traj.transforms[0].rotation.z = 0;
            land_traj.transforms[0].rotation.w = 1;

            land_traj.velocities[0].linear.x = 0;
            land_traj.velocities[0].linear.y = 0;
            land_traj.velocities[0].linear.z = -5;
            land_traj.velocities[0].angular.x = 0;
            land_traj.velocities[0].angular.y = 0;
            land_traj.velocities[0].angular.z = 0;

            land_traj.accelerations[0].linear.x = 0;
            land_traj.accelerations[0].linear.y = 0;
            land_traj.accelerations[0].linear.z = 0;
            land_traj.accelerations[0].angular.x = 0;
            land_traj.accelerations[0].angular.y = 0;
            land_traj.accelerations[0].angular.z = 0;
            
            traj_pub.publish(land_traj);     
            // ros::shutdown();
          } break;
          // ros::shutdown();
      } // end switch
      last_transition_time = ros::Time::now();
    } // end if
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


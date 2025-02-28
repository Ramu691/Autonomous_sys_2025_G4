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
#include <std_msgs/Float32MultiArray.h>
#include <vector>



// Define mission states.
enum RobotState { IDLE, TAKEOFF, NAVIGATE, EXPLORE, LAND };

// Global mission state and current feedback.
RobotState current_state = IDLE;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Point latest_frontier_point;
std::vector<std::vector<float>> stored_lantern_positions;

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
  
}

// Function to store the lantern positions for further use
void updateLanternPositions(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Ensure the data size is a multiple of 3 (x, y, z for each lantern)
    if (msg->data.size() % 3 == 0) {
        // Clear the existing data
        stored_lantern_positions.clear();

        // Store positions as groups of 3 (x, y, z)
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            std::vector<float> lantern = {msg->data[i], msg->data[i + 1], msg->data[i + 2]};
            stored_lantern_positions.push_back(lantern);
        }

        // Print the updated stored positions
        // ROS_INFO("Stored Lantern Positions:");
        // if(current_state == LAND){
        //   for (size_t i = 0; i < stored_lantern_positions.size(); ++i) {
        //       const auto& lantern = stored_lantern_positions[i];
        //       ROS_INFO("Lantern %zu: x: %.2f, y: %.2f, z: %.2f", i + 1, lantern[0], lantern[1], lantern[2]);
        //   }
        // }
    } else {
        ROS_WARN("Received data size is not a multiple of 3. Ignoring.");
    }
}

void num_lantern_callback(const std_msgs::Int16::ConstPtr& msg) {
        num_of_lantern.data = msg->data;  
        //ROS_INFO("Received number of lanterns: %d", num_of_lantern.data);       
}

void printLanternCoordinates(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    int data_length = msg->data.size();
    if (current_state == LAND){

    // Check if the data length is a multiple of 3 (as each lantern has 3 coordinates)
      if (data_length % 3 != 0 ) {
          ROS_ERROR("Received data size is not a multiple of 3.");
          return;
      }

      // Loop through the data and print the coordinates (x, y, z)
      int num_lanterns = data_length / 3;
      for (int i = 0; i < num_lanterns; ++i) {
          int x_idx = i * 3;
          int y_idx = i * 3 + 1;
          int z_idx = i * 3 + 2;

          ROS_INFO("Lantern %d:", i + 1);
          ROS_INFO("  x: %.2f, y: %.2f, z: %.2f", msg->data[x_idx], msg->data[y_idx], msg->data[z_idx]);
      }
    }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh;
  num_of_lantern.data=0;
  
  // Publishers.
  ros::Publisher stm_mode_pub = nh.advertise<std_msgs::String>("/stm_mode", 10);
  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);

  // Subscribers.
  ros::Subscriber current_state_sub = nh.subscribe("/current_state_est", 10, currentStateCallback);
  ros::Subscriber frontier_goal_sub = nh.subscribe("/frontier_goal", 1, frontierGoalCallback);
  ros::Subscriber lantern_goal_sub = nh.subscribe("/num_lanterns", 1, num_lantern_callback);
  ros::Subscriber lantern_sub = nh.subscribe("/lantern_positions", 10, updateLanternPositions);
  
  
  

  // Define waypoints.
  geometry_msgs::PoseStamped cave_entrance_goal;
  cave_entrance_goal.pose.position.x = -321.0;
  cave_entrance_goal.pose.position.y = 10.0;
  cave_entrance_goal.pose.position.z = 15.0;
  cave_entrance_goal.pose.orientation.w = 3.14;
  
  geometry_msgs::PoseStamped take_off;
  take_off.pose.position.x = -38;
  take_off.pose.position.y = 10;
  take_off.pose.position.z = 12;
  // take_off.pose.orientation.w = 1.0;

  ros::Rate rate(5);
  ros::Time last_transition_time = ros::Time::now();
  ros::Time state_entry_time = ros::Time::now();
  ros::Time last_reset_time;  // Time tracking for counter reset
  const double RESET_INTERVAL = 50.0;  // Reset every 5 seconds

  // Transition parameters.
  double takeoff_altitude = 7.0;
  double takeoff_threshold = 5.0;
  double frontier_threshold =0.7;
  double cave_threshold = 5.0;  // Increased threshold so the state transitions when within 5 m of the cave entrance.

  
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
            
            if(sqrt(pow(current_pose.pose.position.x - latest_frontier_point.x, 2) + 
                    pow(current_pose.pose.position.y - latest_frontier_point.y, 2) + 
                    pow(current_pose.pose.position.z - latest_frontier_point.z, 2)) <= frontier_threshold ){
                    frontier_check_counter++;  // Increment counter
                    ROS_INFO("frontier_check_counter: %.2d", frontier_check_counter);                    

                    if (frontier_check_counter >= REQUIRED_COUNT) {
                          frontier_goal_received = false;
                          ROS_INFO("Frontier goal disabled after 20 consecutive checks.");
                      }
                      
                    }
            
            if (!frontier_goal_received && num_of_lantern.data == 5 ) {
                ROS_INFO("Landing.");
                current_state = LAND;
                ROS_INFO("State: LAND");
                for (size_t i = 0; i < stored_lantern_positions.size(); ++i) {
                  const auto& lantern = stored_lantern_positions[i];
                  ROS_INFO("Lantern %zu: x: %.2f, y: %.2f, z: %.2f", i + 1, lantern[0], lantern[1], lantern[2]);
              }
            }

          } break;
        case LAND: {
            //ROS_INFO("State: LAND");
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
            
          } break;
          
      } // end switch
      last_transition_time = ros::Time::now();
    } // end if
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


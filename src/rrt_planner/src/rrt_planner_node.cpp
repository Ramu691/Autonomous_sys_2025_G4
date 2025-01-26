#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/ScopedState.h>
#include <std_msgs/String.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OMPLPathPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber start_pose_sub_, goal_pose_sub_, octomap_sub_, state_sub_;
    ros::Publisher path_pub_;

    octomap::OcTree *map_; // Octomap for obstacle detection
    geometry_msgs::PoseStamped start_pose_, goal_pose_;
    bool start_received_, goal_received_, octomap_received_;
    std::string stm_state_="Explore Cave";

    double map_resolution_;
    double step_size_; // Step size for the planner

public:
    OMPLPathPlanner() : map_(nullptr), start_received_(false), goal_received_(false), octomap_received_(false), step_size_(1.0)
    {
        // Subscribers
        start_pose_sub_ = nh_.subscribe("/pose_est", 1, &OMPLPathPlanner::startPoseCallback, this);
        goal_pose_sub_ = nh_.subscribe("/goal", 1, &OMPLPathPlanner::goalPoseCallback, this);
        octomap_sub_ = nh_.subscribe("octomap_binary", 1, &OMPLPathPlanner::octomapCallback, this);
        //state_sub_ = nh_.subscribe("/Current_State_stm", 1, &OMPLPathPlanner::stateCallback, this);

        // Publisher
        path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path", 1);

        ROS_INFO("OMPL Path Planner initialized.");
    }

    ~OMPLPathPlanner()
    {
        if (map_)
            delete map_;
    }

    // Callback to receive the robot's current state
    // void stateCallback(const std_msgs::String::ConstPtr &msg)
    // {
    //     stm_state_ = msg->data;
    // }

    // Callback to receive the start pose
    void startPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        start_pose_ = *msg;
        start_received_ = true;
        ROS_INFO("Received start pose.");
        attemptPlanning();
    }

    // Callback to receive the goal pose
    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        goal_pose_ = *msg;
        goal_received_ = true;
        ROS_INFO("Received goal pose.");
        attemptPlanning();
    }

    // Callback to receive the octomap
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        if (map_)
            delete map_;

        octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
        map_ = dynamic_cast<octomap::OcTree *>(tree);
        if (map_)
        {
            map_resolution_ = map_->getResolution();
            octomap_received_ = true;
            ROS_INFO("Received octomap.");
            attemptPlanning();
        }
        else
        {
            ROS_WARN("Failed to parse octomap.");
        }
    }

    // Check if a state is valid (collision-free)
    bool isStateValid(const ob::State *state)
    {
        if (!map_)
            return false;

        const auto *state3D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];

        octomap::OcTreeNode *node = map_->search(x, y, z);
        return node == nullptr || node->getOccupancy() <= 0.5;
    }

    // Attempt to plan a path if all data is available
    void attemptPlanning()
    {
        if (start_received_ && goal_received_ && octomap_received_ && stm_state_ == "Explore Cave")
        {
            ROS_INFO("Attempting to plan a path...");
            planPath();
        }
    }

    // Plan the path using OMPL
    void planPath()
    {
        // Define a 3D state space
        auto space = std::make_shared<ob::RealVectorStateSpace>(3);

        // Set bounds based on the octomap
        ob::RealVectorBounds bounds(3);
        double x_min, y_min, z_min, x_max, y_max, z_max;
        map_->getMetricMin(x_min, y_min, z_min);
        map_->getMetricMax(x_max, y_max, z_max);
        bounds.setLow(0, x_min);
        bounds.setHigh(0, x_max);
        bounds.setLow(1, y_min);
        bounds.setHigh(1, y_max);
        bounds.setLow(2, z_min);
        bounds.setHigh(2, z_max);
        space->setBounds(bounds);

        // Create the SimpleSetup object
        og::SimpleSetup ss(space);

        // Set state validity checker
        ss.setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        // Define start and goal states
        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_pose_.pose.position.x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_pose_.pose.position.y;
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_pose_.pose.position.z;

        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_pose_.pose.position.x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_pose_.pose.position.y;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_pose_.pose.position.z;

        ss.setStartAndGoalStates(start, goal);

        // Set the RRT planner
        auto planner = std::make_shared<og::RRT>(ss.getSpaceInformation());
        ss.setPlanner(planner);

        // Solve the planning problem
        ob::PlannerStatus solved = ss.solve(2.0); // 2 seconds timeout

        if (solved)
        {
            ROS_INFO("Path found!");
            ss.simplifySolution();

            // Convert the solution path to a ROS message
            og::PathGeometric path = ss.getSolutionPath();
            nav_msgs::Path ros_path;
            ros_path.header.frame_id = "world";
            ros_path.header.stamp = ros::Time::now();

            for (auto state : path.getStates())
            {
                const auto *s = state->as<ob::RealVectorStateSpace::StateType>();

                geometry_msgs::PoseStamped pose;
                pose.header = ros_path.header;
                pose.pose.position.x = s->values[0];
                pose.pose.position.y = s->values[1];
                pose.pose.position.z = s->values[2];
                pose.pose.orientation.w = 1.0;

                ros_path.poses.push_back(pose);
            }

            path_pub_.publish(ros_path);
        }
        else
        {
            ROS_WARN("No path found.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    OMPLPathPlanner planner;
    ros::spin();
    return 0;
}

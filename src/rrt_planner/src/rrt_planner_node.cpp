#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/ScopedState.h>
#include <std_msgs/String.h>
#include <custom_msgs/FrontierGoalMsg.h>
#include <custom_msgs/CheckPath.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OMPLPathPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber start_pose_sub_, goal_pose_sub_, octomap_sub_;
    ros::Publisher path_pub_;
    ros::ServiceServer check_path_;

    octomap::OcTree *map_;
    geometry_msgs::PoseStamped start_pose_;
    custom_msgs::FrontierGoalMsg goal_pose_;
    bool start_received_, goal_received_, octomap_received_;
    std::string stm_state_ = "Explore Cave";

    double map_resolution_;
    double step_size_factor_; // Step size factor
    double bias_; // Bias for the planner
    double timeout_; 

public:
    OMPLPathPlanner() : map_(nullptr), start_received_(false), goal_received_(false), octomap_received_(false)
    {
        start_pose_sub_ = nh_.subscribe("/pose_est", 1, &OMPLPathPlanner::startPoseCallback, this);
        goal_pose_sub_ = nh_.subscribe("/frontier_goal", 1, &OMPLPathPlanner::goalPoseCallback, this);
        octomap_sub_ = nh_.subscribe("octomap_binary", 1, &OMPLPathPlanner::octomapCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path", 1);

        check_path_ = nh_.advertiseService("check_path", &OMPLPathPlanner::handleCheckPath, this);

        if (!ros::param::get("step_size_factor", step_size_factor_)) ROS_FATAL("Required parameter step_size_factor was not found on parameter server");
        if (!ros::param::get("bias", bias_)) ROS_FATAL("Required parameter bias was not found on parameter server");
        if (!ros::param::get("timeout", timeout_)) ROS_FATAL("Required parameter timeout was not found on parameter server");
        ROS_INFO("OMPL Path Planner initialized.");
    }

    ~OMPLPathPlanner()
    {
        if (map_)
            delete map_;
    }

    bool handleCheckPath(custom_msgs::CheckPath::Request &req,
                        custom_msgs::CheckPath::Response &res) {
        // save original goal
        custom_msgs::FrontierGoalMsg real_goal = goal_pose_;
        goal_pose_ = req.goal_pos;
        goal_received_ = true;
        res.pathFound = planPath(true);
        // restore original goal
        goal_pose_ = real_goal;;
        return true;
    }

    void startPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        start_pose_ = *msg;
        start_received_ = true;
        attemptPlanning();
    }

    void goalPoseCallback(const custom_msgs::FrontierGoalMsg::ConstPtr &frontiergoal)
    {
        goal_pose_ = *frontiergoal;
        goal_received_ = true;
        attemptPlanning();
    }

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
            attemptPlanning();
        }
        else
        {
            ROS_WARN("Failed to parse octomap.");
        }
    }

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

    void attemptPlanning()
    {
        if (start_received_ && goal_received_ && octomap_received_ && stm_state_ == "Explore Cave")
        {
            ROS_INFO("Attempting to plan a path...");
            planPath();
        }
    }

    bool planPath(){
        return planPath(false);
    }

    bool planPath(bool onlyCheck)
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(3);
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

        if (start_pose_.pose.position.x < x_min || start_pose_.pose.position.x > x_max ||
            start_pose_.pose.position.y < y_min || start_pose_.pose.position.y > y_max ||
            start_pose_.pose.position.z < z_min || start_pose_.pose.position.z > z_max ) {
            ROS_ERROR("Start position is out of bounds!");
            return false;
        }

        if (goal_pose_.point.x < x_min || goal_pose_.point.x > x_max ||
            goal_pose_.point.y < y_min || goal_pose_.point.y > y_max ||
            goal_pose_.point.z < z_min || goal_pose_.point.z > z_max ) {
            ROS_ERROR("Goal position is out of bounds!");
            return false;
        }

        og::SimpleSetup ss(space);
        ss.setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        ob::ScopedState<> start(space), goal(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_pose_.pose.position.x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_pose_.pose.position.y;
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_pose_.pose.position.z;

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_pose_.point.x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_pose_.point.y;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_pose_.point.z;

        ss.setStartAndGoalStates(start, goal);
        auto planner = std::make_shared<og::RRT>(ss.getSpaceInformation());
        planner->setRange(step_size_factor_ * map_resolution_);
        planner->setGoalBias(bias_);
        ss.setPlanner(planner);

        // Solve the planning problem
        ob::PlannerStatus solved = ss.solve(timeout_);

        if(onlyCheck){
            return solved;
        }

        if (solved)
        {
            ROS_INFO("Path found!");
            // ss.simplifySolution();

            og::PathGeometric path = ss.getSolutionPath();
            nav_msgs::Path ros_path;
            ros_path.header.frame_id = "world";

            for (auto state : path.getStates())
            {
                const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = s->values[0];
                pose.pose.position.y = s->values[1];
                pose.pose.position.z = s->values[2];

                // check again with raytracing as aditional robustnes
                if(!ros_path.poses.empty()){                    
                    octomap::point3d p1(ros_path.poses[-1].pose.position.x, ros_path.poses[-1].pose.position.y, ros_path.poses[-1].pose.position.z);
                    octomap::point3d p2(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                    octomap::point3d hit;
                    double distance = computeDistance(ros_path.poses[-1].pose.position, pose.pose.position);
                    if(map_->castRay(p1, p2 - p1, hit, true, distance)){
                        // not successfull
                        return false;
                    }
                }

                ros_path.poses.push_back(pose);
            }

            path_pub_.publish(ros_path);
        }
        else
        {
            ROS_WARN("No path found.");
        }
        return solved;
    }

    double computeDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    OMPLPathPlanner planner;
    ros::spin();
    return 0;
}

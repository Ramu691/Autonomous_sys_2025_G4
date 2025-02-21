/****************************************************
 *  traj_gen_conditional_switch.cpp
 *  
 *  Subscribes: /rrt_path (nav_msgs::Path)
 *  Publishes:  /trajectory (MultiDOFJointTrajectoryPoint)
 ****************************************************/
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <std_msgs/String.h>
// For converting yaw->quaternion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for toMsg()

/**
 * @brief Simple function to compute total Euclidean distance of a Path
 */
double computePathDistance(const nav_msgs::Path &path_msg)
{
    double dist = 0.0;
    for (size_t i = 0; i + 1 < path_msg.poses.size(); ++i)
    {
        auto &p1 = path_msg.poses[i].pose.position;
        auto &p2 = path_msg.poses[i+1].pose.position;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        dist += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return dist;
}

/**
 * @brief Compute the 8 polynomial coefficients for a single segment
 *        with zero velocity/acc/jerk at t=0 and t=T.
 */
Eigen::VectorXd computeSegmentCoeffs(double p0, double pT, double T)
{
    Eigen::Matrix<double, 8, 8> A; 
    A.setZero();
    Eigen::Matrix<double, 8, 1> b; 
    b.setZero();

    // Boundary conditions at t=0
    A(0,0) = 1.0;       b(0) = p0;  
    A(1,1) = 1.0;       b(1) = 0.0;  
    A(2,2) = 2.0;       b(2) = 0.0;  
    A(3,3) = 6.0;       b(3) = 0.0;  

    // Boundary conditions at t=T
    for(int i=0; i<8; i++){
        A(4,i) = std::pow(T, i);  
    }
    b(4) = pT;

    for(int i=1; i<8; i++){
        A(5,i) = i * std::pow(T, i-1);
    }
    b(5) = 0.0;

    for(int i=2; i<8; i++){
        A(6,i) = i*(i-1)*std::pow(T, i-2);
    }
    b(6) = 0.0;

    for(int i=3; i<8; i++){
        A(7,i) = i*(i-1)*(i-2)*std::pow(T, i-3);
    }
    b(7) = 0.0;

    return A.colPivHouseholderQr().solve(b);
}

double evalPoly(const Eigen::VectorXd &coeffs, double t, int derivative_order)
{
    double val = 0.0;
    for(int i = derivative_order; i < 8; i++){
        double factor = 1.0;
        for(int k = 0; k < derivative_order; k++){
            factor *= (i - k);
        }
        val += coeffs[i] * factor * std::pow(t, i - derivative_order);
    }
    return val;
}

class TrajGen
{
public:
    TrajGen()
      : nh_("~"),
        publish_rate_(20.0),
        has_coeffs_(false),
        total_time_(0.0),
        old_path_distance_(0.0),
        distance_margin_factor_(0.3)
    {
        /*
         * distance_margin_factor_ is used to decide if new path is significantly better:
         * If new_path_dist < old_path_dist_remaining * distance_margin_factor_,
         * we accept new path. (For example, 0.7 => at least 30% improvement)
         *
         */

        path_sub_ = nh_.subscribe("/rrt_path", 1, &TrajGen::pathCallback, this);
        traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/trajectory", 10);
        state_subscriber_ = nh_.subscribe("/stm_mode", 1, &TrajGen::onStateStm, this);

        timer_ = nh_.createTimer(
            ros::Duration(1.0/publish_rate_),
            &TrajGen::timerCallback,
            this
        );

        ROS_INFO("TrajGen: multi-segment polynomial with conditional path switching.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_, state_subscriber_;
    ros::Publisher  traj_pub_;
    ros::Timer      timer_;
    std::string stm_state_ ;

    double publish_rate_;

    // Piecewise polynomial data
    std::vector<Eigen::VectorXd> cxs_, cys_, czs_;
    std::vector<double> segTimes_;
    std::vector<double> cumTimes_;
    double total_time_;
    ros::Time start_time_;
    bool has_coeffs_;

    // For comparing old vs. new path
    double old_path_distance_;            // total distance of the *original* path
    double distance_margin_factor_;       // threshold for switching mid-flight

private:

    /**
     * @brief Called when a new /rrt_path arrives.
     *        We conditionally accept if significantly better or if we finished old trajectory.
     */
    void pathCallback(const nav_msgs::Path &path_msg)
    {
        // Basic checks
        size_t N = path_msg.poses.size();
        if (N < 2) {
            ROS_WARN("TrajGen: /rrt_path has <2 poses, ignoring.");
            return;
        }

        // Compute new path's total distance
        double new_path_dist = computePathDistance(path_msg);

        if (!has_coeffs_)
        {
            // If we are not currently executing any trajectory, just accept the new path
            buildTrajectory(path_msg);
            old_path_distance_ = new_path_dist;
            ROS_INFO("TrajGen: Accepting new path (no old trajectory in progress). Dist=%.1f", new_path_dist);
            return;
        }

        // We *do* have a trajectory in progress. Let’s see if it's done or if the new path is better
        double t_elapsed = (ros::Time::now() - start_time_).toSec();
        if (t_elapsed >= total_time_) {
            // Old trajectory finished, so accept the new one
            buildTrajectory(path_msg);
            old_path_distance_ = new_path_dist;
            ROS_INFO("TrajGen: Old trajectory done. Accepting new path. Dist=%.1f", new_path_dist);
            return;
        }

        // If we are mid-flight, check if the new path is significantly better
        // 1) Distance traveled so far? We don’t know precisely, but we can approximate
        //    how much "old_path" distance remains. Alternatively, store each segment’s distance.
        // For simplicity, assume old_path_distance_ is the total. We'll guess we haven't traveled
        // that much yet. A more advanced method would track distance traveled so far.
        double old_path_dist_remaining = old_path_distance_ * (1.0 - (t_elapsed / total_time_));
        // ^ crude linear approximation

        if (new_path_dist < old_path_dist_remaining * distance_margin_factor_)
        {
            // The new path is significantly shorter than the remainder of the old path
            // => Switch to the new path
            ROS_WARN("TrajGen: Found a better path mid-flight! Old remaining=%.1f, new=%.1f => SWITCHING",
                     old_path_dist_remaining, new_path_dist);
            buildTrajectory(path_msg);
            old_path_distance_ = new_path_dist;
        }
        else
        {
            // Otherwise ignore it
            ROS_INFO("TrajGen: Mid-flight ignoring new path (Dist=%.1f not better than old=%.1f).",
                     new_path_dist, old_path_dist_remaining);
        }
    }

    void onStateStm(const std_msgs::String& cur_state){
      stm_state_ = cur_state.data;
    }

    /**
     * @brief Actually build the multi-segment polynomial from the path message.
     */
    void buildTrajectory(const nav_msgs::Path &path_msg)
    {
        size_t N = path_msg.poses.size();
        if (N < 2) {
            ROS_WARN("TrajGen: Can't build trajectory, <2 waypoints.");
            has_coeffs_ = false;
            total_time_ = 0.0;
            return;
        }

        cxs_.clear();
        cys_.clear();
        czs_.clear();
        segTimes_.clear();
        cumTimes_.clear();
        total_time_ = 0.0;

        for (size_t i = 0; i < N - 1; ++i)
        {
            auto startP = path_msg.poses[i].pose.position;
            auto endP   = path_msg.poses[i+1].pose.position;

            double dx = endP.x - startP.x;
            double dy = endP.y - startP.y;
            double dz = endP.z - startP.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // simple time rule
            double Tseg = std::max(1.0, dist * 0.3);

            Eigen::VectorXd cx = computeSegmentCoeffs(startP.x, endP.x, Tseg);
            Eigen::VectorXd cy = computeSegmentCoeffs(startP.y, endP.y, Tseg);
            Eigen::VectorXd cz = computeSegmentCoeffs(startP.z, endP.z, Tseg);

            cxs_.push_back(cx);
            cys_.push_back(cy);
            czs_.push_back(cz);
            segTimes_.push_back(Tseg);
        }

        cumTimes_.resize(segTimes_.size() + 1, 0.0);
        for (size_t i = 0; i < segTimes_.size(); ++i) {
            cumTimes_[i+1] = cumTimes_[i] + segTimes_[i];
        }
        total_time_ = cumTimes_.back();

        has_coeffs_ = true;
        start_time_ = ros::Time::now();

        double dist = computePathDistance(path_msg);
        ROS_INFO("TrajGen: Built %lu segments. total_time=%.2f, path_dist=%.2f",
                 segTimes_.size(), total_time_, dist);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!has_coeffs_)
            return;

        double t_elapsed = (ros::Time::now() - start_time_).toSec();
        if (t_elapsed >= total_time_) {
            // hold final
            publishPoint(segTimes_.size() - 1, segTimes_.back());
            ROS_INFO_THROTTLE(2.0, "TrajGen: Holding final waypoint at t=%.2f", t_elapsed);
            return;
        }

        // find segment
        size_t seg_idx = 0;
        for (size_t i = 0; i < segTimes_.size(); ++i) {
            if (t_elapsed < cumTimes_[i+1]) {
                seg_idx = i;
                break;
            }
        }

        double t_in_segment = t_elapsed - cumTimes_[seg_idx];
        publishPoint(seg_idx, t_in_segment);
    }

    void publishPoint(size_t seg_idx, double t_in_segment)
    {
        const Eigen::VectorXd &cx = cxs_[seg_idx];
        const Eigen::VectorXd &cy = cys_[seg_idx];
        const Eigen::VectorXd &cz = czs_[seg_idx];
        double Tseg = segTimes_[seg_idx];

        if (t_in_segment > Tseg) t_in_segment = Tseg;

        double px = evalPoly(cx, t_in_segment, 0);
        double py = evalPoly(cy, t_in_segment, 0);
        double pz = evalPoly(cz, t_in_segment, 0);

        double vx = evalPoly(cx, t_in_segment, 1);
        double vy = evalPoly(cy, t_in_segment, 1);
        double vz = evalPoly(cz, t_in_segment, 1);

        double ax = evalPoly(cx, t_in_segment, 2);
        double ay = evalPoly(cy, t_in_segment, 2);
        double az = evalPoly(cz, t_in_segment, 2);

        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.velocities.resize(1);
        msg.accelerations.resize(1);

        msg.transforms[0].translation.x = px;
        msg.transforms[0].translation.y = py;
        msg.transforms[0].translation.z = pz;

        double heading = std::atan2(vy, vx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, heading);
        q.normalize();
        msg.transforms[0].rotation = tf2::toMsg(q);

        msg.velocities[0].linear.x = vx;
        msg.velocities[0].linear.y = vy;
        msg.velocities[0].linear.z = vz;

        msg.accelerations[0].linear.x = ax;
        msg.accelerations[0].linear.y = ay;
        msg.accelerations[0].linear.z = az;

        traj_pub_.publish(msg);

        if (stm_state_=="LAND"){
            ros::shutdown();
        }

        ROS_INFO_THROTTLE(1.0,
            "TrajGen: seg=%lu t=%.2f/%.2f -> pos(%.2f,%.2f,%.2f) vel(%.2f,%.2f,%.2f) yaw=%.1f deg",
            seg_idx, t_in_segment, Tseg, px, py, pz, vx, vy, vz, heading * 180.0 / M_PI
        );
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_gen_conditional_switch");
    TrajGen node;
    ros::spin();
    return 0;
}

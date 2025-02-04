/****************************************************
 *  traj_gen.cpp (Single segment: first pose to last pose)
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

// For converting yaw->quaternion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for toMsg()

/**
 * @brief Compute the 8 polynomial coefficients for a single segment:
 *        p(0)=p0, p'(0)=0, p''(0)=0, p'''(0)=0
 *        p(T)=pT, p'(T)=0, p''(T)=0, p'''(T)=0
 */
Eigen::VectorXd computeSegmentCoeffs(double p0, double pT, double T)
{
    Eigen::Matrix<double, 8, 8> A; 
    A.setZero();
    Eigen::Matrix<double, 8, 1> b; 
    b.setZero();

    // Boundary conditions at t=0
    A(0,0) = 1.0;       b(0) = p0;   // p(0)=p0
    A(1,1) = 1.0;       b(1) = 0.0;  // p'(0)=0
    A(2,2) = 2.0;       b(2) = 0.0;  // p''(0)=0
    A(3,3) = 6.0;       b(3) = 0.0;  // p'''(0)=0

    // Boundary conditions at t=T
    for(int i=0; i<8; i++){
        A(4,i) = std::pow(T, i);  // p(T)=pT
    }
    b(4) = pT;

    for(int i=1; i<8; i++){
        A(5,i) = i * std::pow(T, i-1); // p'(T)=0
    }
    b(5) = 0.0;

    for(int i=2; i<8; i++){
        A(6,i) = i*(i-1)*std::pow(T, i-2); // p''(T)=0
    }
    b(6) = 0.0;

    for(int i=3; i<8; i++){
        A(7,i) = i*(i-1)*(i-2)*std::pow(T, i-3); // p'''(T)=0
    }
    b(7) = 0.0;

    return A.colPivHouseholderQr().solve(b);
}

/**
 * @brief Evaluate polynomial or derivative at time t
 *  derivative_order=0 => position,1=>velocity,2=>acc,3=>jerk
 */
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
      : nh_("~"), publish_rate_(20.0), has_coeffs_(false)
    {
        path_sub_ = nh_.subscribe("/rrt_path", 1, &TrajGen::pathCallback, this);
        traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/trajectory", 10);

        timer_ = nh_.createTimer(
            ros::Duration(1.0/publish_rate_),
            &TrajGen::timerCallback,
            this
        );

        ROS_INFO("TrajGen: Single-segment from first pose to last pose. Subscribing /rrt_path.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher  traj_pub_;
    ros::Timer      timer_;

    double publish_rate_;

    // Polynomial coefficients
    Eigen::VectorXd cx_, cy_, cz_;
    double T_;                     // total time for the single segment
    ros::Time start_time_;
    bool has_coeffs_;

private:

    /**
     * @brief Subscribes to /rrt_path, uses only first & last poses for a single segment
     *        BUT ignores new path messages while still executing a trajectory.
     */
    void pathCallback(const nav_msgs::Path &path_msg)
    {
        // If we are already executing a trajectory and haven't finished, ignore new paths
        if (has_coeffs_) {
            double t_elapsed = (ros::Time::now() - start_time_).toSec();
            if (t_elapsed < T_) {
                ROS_INFO("Ignoring new path; still executing old trajectory (t=%.2f / T=%.2f).", t_elapsed, T_);
                return;
            }
        }

        size_t N = path_msg.poses.size();
        if (N < 2) {
            ROS_WARN("TrajGen: /rrt_path has <2 poses, no trajectory generated.");
            return;
        }

        // Use only the first pose and the final pose
        auto startP = path_msg.poses.front().pose.position;
        auto endP   = path_msg.poses.back().pose.position;

        // Log them
        ROS_INFO("TrajGen: Generating single-segment from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)",
                 startP.x, startP.y, startP.z,
                 endP.x,   endP.y,   endP.z);

        double dx = endP.x - startP.x;
        double dy = endP.y - startP.y;
        double dz = endP.z - startP.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // Lower the minimum time so short distances go faster
        T_ = std::max(1.0, dist * 0.3);

        // Compute polynomials along x, y, z
        cx_ = computeSegmentCoeffs(startP.x, endP.x, T_);
        cy_ = computeSegmentCoeffs(startP.y, endP.y, T_);
        cz_ = computeSegmentCoeffs(startP.z, endP.z, T_);

        has_coeffs_ = true;
        start_time_ = ros::Time::now();

        ROS_INFO("TrajGen: Built single polynomial. dist=%.2f, T=%.2f", dist, T_);
    }

    /**
     * @brief Timer callback: sample polynomial at 20 Hz
     */
    void timerCallback(const ros::TimerEvent&)
    {
        if (!has_coeffs_) {
            // No polynomial yet
            return;
        }
        double t_elapsed = (ros::Time::now() - start_time_).toSec();
        if (t_elapsed >= T_) {
            // Hold final point if we exceed final time
            publishPoint(T_);
            ROS_INFO_THROTTLE(2.0, "TrajGen: Holding final waypoint at t=%.2f", t_elapsed);
            return;
        }
        // Otherwise, sample at the current time
        publishPoint(t_elapsed);
    }

    /**
     * @brief Evaluate polynomials & publish the MultiDOFJointTrajectoryPoint
     *        Includes a basic heading from velocity (atan2(vy, vx)).
     */
    void publishPoint(double t)
    {
        // Evaluate polynomial for position
        double px = evalPoly(cx_, t, 0);
        double py = evalPoly(cy_, t, 0);
        double pz = evalPoly(cz_, t, 0);

        // Evaluate polynomial for velocity
        double vx = evalPoly(cx_, t, 1);
        double vy = evalPoly(cy_, t, 1);
        double vz = evalPoly(cz_, t, 1);

        // Evaluate polynomial for acceleration
        double ax = evalPoly(cx_, t, 2);
        double ay = evalPoly(cy_, t, 2);
        double az = evalPoly(cz_, t, 2);

        // Build the trajectory point
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.velocities.resize(1);
        msg.accelerations.resize(1);

        // Position
        msg.transforms[0].translation.x = px;
        msg.transforms[0].translation.y = py;
        msg.transforms[0].translation.z = pz;

        // Compute a yaw from velocity
        double heading = std::atan2(vy, vx);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, heading);  // (roll=0, pitch=0, yaw=heading)
        q.normalize();
        msg.transforms[0].rotation = tf2::toMsg(q);

        // Velocity
        msg.velocities[0].linear.x = vx;
        msg.velocities[0].linear.y = vy;
        msg.velocities[0].linear.z = vz;

        // Acceleration
        msg.accelerations[0].linear.x = ax;
        msg.accelerations[0].linear.y = ay;
        msg.accelerations[0].linear.z = az;

        // Publish
        traj_pub_.publish(msg);

        // Optional debug info
        ROS_INFO_THROTTLE(1.0,
            "TrajGen: t=%.2f -> pos(%.2f,%.2f,%.2f) vel(%.2f,%.2f,%.2f), yaw=%.2f deg",
            t, px, py, pz, vx, vy, vz, heading * 180.0 / M_PI
        );
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_gen_single_segment");
    TrajGen node;
    ros::spin();
    return 0;
}

/****************************************************
 *  traj_gen.cpp
 *  
 *  Subscribes: /rrt_path (nav_msgs::Path)
 *  Publishes:  /trajectory (MultiDOFJointTrajectoryPoint)
 *
 *  This version uses cubic polynomials that ensure 
 *  continuous motion (nonzero velocity at waypoints) by 
 *  computing a velocity at each waypoint via finite differences.
 ****************************************************/

 #include <ros/ros.h>
 #include <nav_msgs/Path.h>
 #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
 #include <Eigen/Dense>
 #include <cmath>
 #include <vector>
 #include <std_msgs/String.h>
 #include <geometry_msgs/PoseStamped.h>
 
 // For converting yaw->quaternion
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for toMsg()
 
 // Cave entrance coordinates (from original waypoints)
 #define CAVE_X -324.0
 #define CAVE_Y 10.0
 #define CAVE_Z 16.0
 #define CAVE_YAW 3.14
 
 /**
  * @brief Compute the total Euclidean distance along a path.
  */
 double computePathDistance(const nav_msgs::Path &path_msg)
 {
     double dist = 0.0;
     for (size_t i = 0; i + 1 < path_msg.poses.size(); ++i)
     {
         const auto &p1 = path_msg.poses[i].pose.position;
         const auto &p2 = path_msg.poses[i+1].pose.position;
         double dx = p2.x - p1.x;
         double dy = p2.y - p1.y;
         double dz = p2.z - p1.z;
         dist += std::sqrt(dx*dx + dy*dy + dz*dz);
     }
     return dist;
 }
 
 /**
  * @brief Compute the 4 coefficients of a cubic polynomial for one segment.
  * 
  * The cubic polynomial is defined as:
  *   p(t) = a0 + a1 t + a2 t^2 + a3 t^3,
  * with the constraints:
  *   p(0) = p0,   p(T) = pT,
  *   p'(0)= v0,   p'(T) = vT.
  *
  * The solution is given in closed-form.
  */
 Eigen::VectorXd computeCubicCoeffs(double p0, double pT, double T, double v0, double vT)
 {
     Eigen::VectorXd coeffs(4);
     coeffs[0] = p0;
     coeffs[1] = v0;
     
     // Define A and B for simplicity
     double A = pT - p0 - v0 * T;
     double B = vT - v0;
     
     // Solve:
     //   a2 * T^2 + a3 * T^3 = A
     //   2*a2*T + 3*a3*T^2 = B
     coeffs[3] = (B * T - 2.0 * A) / (T * T * T);   // a3 = (B*T - 2A) / T^3
     coeffs[2] = (A - coeffs[3] * T * T * T) / (T * T); // a2 = (A - a3*T^3) / T^2
     
     return coeffs;
 }
 
 /**
  * @brief Evaluate a cubic polynomial or its derivative.
  * 
  * For derivative_order:
  *   0 -> position, 1 -> velocity, 2 -> acceleration, 3 -> jerk.
  */
 double evalCubicPoly(const Eigen::VectorXd &coeffs, double t, int derivative_order)
 {
     double val = 0.0;
     if (derivative_order == 0) {
         // p(t) = a0 + a1*t + a2*t^2 + a3*t^3
         val = coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t;
     }
     else if (derivative_order == 1) {
         // p'(t) = a1 + 2*a2*t + 3*a3*t^2
         val = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t*t;
     }
     else if (derivative_order == 2) {
         // p''(t) = 2*a2 + 6*a3*t
         val = 2*coeffs[2] + 6*coeffs[3]*t;
     }
     else if (derivative_order == 3) {
         // p'''(t) = 6*a3
         val = 6*coeffs[3];
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
          * distance_margin_factor_ is used to decide if a new path is
          * significantly better than the remaining distance of the current one.
          */
 
         path_sub_ = nh_.subscribe("/rrt_path", 1, &TrajGen::pathCallback, this);
         desired_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
         state_sub_ = nh_.subscribe("/stm_mode", 1, &TrajGen::stateCallback, this);
         pose_sub_ = nh_.subscribe("/pose_est", 1, &TrajGen::poseCallback, this);

         timer_ = nh_.createTimer(
             ros::Duration(1.0/publish_rate_),
             &TrajGen::timerCallback,
             this
         );
 
         ROS_INFO("TrajGen: multi-segment cubic polynomial with continuous motion.");
     }
 
 private:
     ros::NodeHandle nh_;
     ros::Subscriber path_sub_, state_sub_, pose_sub_;
     ros::Publisher  desired_pub_;
     ros::Timer      timer_;
     
     geometry_msgs::PoseStamped current_pose_;
     std::string stm_state_;
     bool pose_received_ = false;
 
     // Original trajectory variables
     double publish_rate_;
     std::vector<Eigen::VectorXd> cxs_, cys_, czs_;
     std::vector<double> segTimes_;
     std::vector<double> cumTimes_;
     double total_time_;
     ros::Time start_time_;
     bool has_coeffs_;
     double old_path_distance_;
     double distance_margin_factor_;
 
     void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
         current_pose_ = *msg;
         pose_received_ = true;
     }
 
     void stateCallback(const std_msgs::String& msg) {
         stm_state_ = msg.data;
         
         // Handle state transitions
         if (stm_state_ == "NAVIGATE" && pose_received_) {
             generateCaveEntrancePath();
         }

     }
 
     void generateCaveEntrancePath() {
         nav_msgs::Path path;
         path.header.stamp = ros::Time::now();
         path.header.frame_id = "world";
 
         // Start from current position
         geometry_msgs::PoseStamped start;
         start.pose = current_pose_.pose;
         path.poses.push_back(start);
 
         // Add cave entrance waypoint
         geometry_msgs::PoseStamped cave;
         cave.pose.position.x = CAVE_X;
         cave.pose.position.y = CAVE_Y;
         cave.pose.position.z = CAVE_Z;
         tf2::Quaternion q;
         q.setRPY(0, 0, CAVE_YAW);
         cave.pose.orientation = tf2::toMsg(q);
         path.poses.push_back(cave);
 
         // Trigger path processing
         pathCallback(path);
         ROS_INFO("Generated cave entrance path");
     }
     /**
      * @brief Callback for new /rrt_path messages.
      *        Conditionally switch to the new path if it is significantly better.
      */
     void pathCallback(const nav_msgs::Path &path_msg)
     {

            size_t N = path_msg.poses.size();
         if (N < 2) {
             ROS_WARN("TrajGen: /rrt_path has <2 poses, ignoring.");
             return;
         }
         
         double new_path_dist = computePathDistance(path_msg);
 
         if (!has_coeffs_) {
             buildTrajectory(path_msg);
             old_path_distance_ = new_path_dist;
             ROS_INFO("TrajGen: Accepting new path (no trajectory in progress). Dist=%.1f", new_path_dist);
             return;
         }
 
         double t_elapsed = (ros::Time::now() - start_time_).toSec();
         if (t_elapsed >= total_time_) {
             // Old trajectory finished; accept the new one.
             buildTrajectory(path_msg);
             old_path_distance_ = new_path_dist;
             ROS_INFO("TrajGen: Old trajectory finished. Accepting new path. Dist=%.1f", new_path_dist);
             return;
         }
 
         // Estimate remaining distance (crudely, by linear proportion)
         double old_path_dist_remaining = old_path_distance_ * (1.0 - (t_elapsed / total_time_));
         if (new_path_dist < old_path_dist_remaining * distance_margin_factor_) {
             ROS_WARN("TrajGen: Found a significantly better path mid-flight! Old remaining=%.1f, new=%.1f => SWITCHING",
                      old_path_dist_remaining, new_path_dist);
             buildTrajectory(path_msg);
             old_path_distance_ = new_path_dist;
         }
         else {
             ROS_INFO("TrajGen: Mid-flight ignoring new path (new=%.1f not better than remaining=%.1f).",
                      new_path_dist, old_path_dist_remaining);
         }

        
        
         
     }
 
     /**
      * @brief Build the continuous trajectory from the path.
      * 
      * For each segment between consecutive waypoints, we first compute a segment time.
      * Then we compute a desired velocity at each waypoint using a finite-difference approximation.
      * Finally, we compute cubic coefficients for each segment that satisfy position and velocity
      * constraints at the endpoints.
      */
     void buildTrajectory(const nav_msgs::Path &path_msg)
     {
         size_t N = path_msg.poses.size();
         if (N < 2) {
             ROS_WARN("TrajGen: Cannot build trajectory with <2 waypoints.");
             has_coeffs_ = false;
             total_time_ = 0.0;
             return;
         }
 
         // Clear any previous data.
         cxs_.clear();
         cys_.clear();
         czs_.clear();
         segTimes_.clear();
         cumTimes_.clear();
         total_time_ = 0.0;
 
         // Extract positions and compute segment times.
         std::vector<geometry_msgs::Point> points;
         points.resize(N);
         for (size_t i = 0; i < N; ++i) {
             points[i] = path_msg.poses[i].pose.position;
         }
         // Compute time for each segment (using a simple rule based on distance).
         for (size_t i = 0; i < N - 1; ++i) {
             double dx = points[i+1].x - points[i].x;
             double dy = points[i+1].y - points[i].y;
             double dz = points[i+1].z - points[i].z;
             double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
             // 0.15 is the speed factor -> lower is faster with speed 
             double Tseg = std::max(1.0, dist * 0.15);
             segTimes_.push_back(Tseg);
         }
 
         // Compute cumulative times.
         cumTimes_.resize(segTimes_.size() + 1, 0.0);
         for (size_t i = 0; i < segTimes_.size(); ++i) {
             cumTimes_[i+1] = cumTimes_[i] + segTimes_[i];
         }
         total_time_ = cumTimes_.back();
 
         // Compute desired velocities at waypoints (using finite differences).
         // For each dimension, we create a vector of velocities.
         std::vector<double> vx(N, 0.0), vy(N, 0.0), vz(N, 0.0);
         // For the first waypoint, use forward difference.
         vx[0] = (points[1].x - points[0].x) / segTimes_[0];
         vy[0] = (points[1].y - points[0].y) / segTimes_[0];
         vz[0] = (points[1].z - points[0].z) / segTimes_[0];
         // For the last waypoint, use backward difference.
         vx[N-1] = (points[N-1].x - points[N-2].x) / segTimes_[N-2];
         vy[N-1] = (points[N-1].y - points[N-2].y) / segTimes_[N-2];
         vz[N-1] = (points[N-1].z - points[N-2].z) / segTimes_[N-2];
         // For intermediate waypoints, use the average of forward and backward differences.
         for (size_t i = 1; i < N-1; ++i) {
             double Tprev = segTimes_[i-1];
             double Tnext = segTimes_[i];
             vx[i] = 0.5 * ( (points[i].x - points[i-1].x)/Tprev + (points[i+1].x - points[i].x)/Tnext );
             vy[i] = 0.5 * ( (points[i].y - points[i-1].y)/Tprev + (points[i+1].y - points[i].y)/Tnext );
             vz[i] = 0.5 * ( (points[i].z - points[i-1].z)/Tprev + (points[i+1].z - points[i].z)/Tnext );
         }
 
         // For each segment, compute cubic coefficients for x, y, and z.
         for (size_t i = 0; i < N - 1; ++i)
         {
             double Tseg = segTimes_[i];
             // x dimension
             Eigen::VectorXd cx = computeCubicCoeffs(points[i].x, points[i+1].x, Tseg, vx[i], vx[i+1]);
             // y dimension
             Eigen::VectorXd cy = computeCubicCoeffs(points[i].y, points[i+1].y, Tseg, vy[i], vy[i+1]);
             // z dimension
             Eigen::VectorXd cz = computeCubicCoeffs(points[i].z, points[i+1].z, Tseg, vz[i], vz[i+1]);
             
             cxs_.push_back(cx);
             cys_.push_back(cy);
             czs_.push_back(cz);
         }
 
         has_coeffs_ = true;
         start_time_ = ros::Time::now();
 
         double totalPathDist = computePathDistance(path_msg);
         ROS_INFO("TrajGen: Built %lu segments. Total time = %.2f, Path distance = %.2f", segTimes_.size(), total_time_, totalPathDist);
     }
 
     /**
      * @brief Timer callback: sample the current segment and publish the trajectory point.
      */
     void timerCallback(const ros::TimerEvent&)
     {
         if (!has_coeffs_)
             return;
 
         double t_elapsed = (ros::Time::now() - start_time_).toSec();
         if (t_elapsed >= total_time_) {
             // Hold the final waypoint.
             publishPoint(segTimes_.size() - 1, segTimes_.back());
             ROS_INFO_THROTTLE(2.0, "TrajGen: Holding final waypoint at t=%.2f", t_elapsed);
             return;
         }
 
         // Determine which segment we're in.
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
 
     /**
      * @brief Evaluate the cubic polynomials for the given segment and publish the trajectory point.
      */
     void publishPoint(size_t seg_idx, double t_in_segment)
     {
         // Retrieve cubic coefficients for this segment.
         const Eigen::VectorXd &cx = cxs_[seg_idx];
         const Eigen::VectorXd &cy = cys_[seg_idx];
         const Eigen::VectorXd &cz = czs_[seg_idx];
         double Tseg = segTimes_[seg_idx];
 
         // Clamp time (just in case)
         if (t_in_segment > Tseg) t_in_segment = Tseg;
 
         // Evaluate position.
         double px = evalCubicPoly(cx, t_in_segment, 0);
         double py = evalCubicPoly(cy, t_in_segment, 0);
         double pz = evalCubicPoly(cz, t_in_segment, 0);
 
         // Evaluate velocity.
         double vx = evalCubicPoly(cx, t_in_segment, 1);
         double vy = evalCubicPoly(cy, t_in_segment, 1);
         double vz = evalCubicPoly(cz, t_in_segment, 1);
 
         // Evaluate acceleration (if needed).
         double ax = evalCubicPoly(cx, t_in_segment, 2);
         double ay = evalCubicPoly(cy, t_in_segment, 2);
         double az = evalCubicPoly(cz, t_in_segment, 2);
 
         trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
         msg.transforms.resize(1);
         msg.velocities.resize(1);
         msg.accelerations.resize(1);
 
         // Set position.
         msg.transforms[0].translation.x = px;
         msg.transforms[0].translation.y = py;
         msg.transforms[0].translation.z = pz;
 
         // Compute heading (yaw) from velocity.
         double heading = std::atan2(vy, vx);
         tf2::Quaternion q;
         q.setRPY(0.0, 0.0, heading);
         q.normalize();
         msg.transforms[0].rotation = tf2::toMsg(q);
 
         // Set velocity.
         msg.velocities[0].linear.x = vx;
         msg.velocities[0].linear.y = vy;
         msg.velocities[0].linear.z = vz;
 
         // Set acceleration.
         msg.accelerations[0].linear.x = ax;
         msg.accelerations[0].linear.y = ay;
         msg.accelerations[0].linear.z = az;
 

         desired_pub_.publish(msg);
         if (stm_state_=="LAND"){
            ros::shutdown();
         }
 
         ROS_INFO_THROTTLE(1.0,
             "TrajGen: seg=%lu t=%.2f/%.2f -> pos(%.2f, %.2f, %.2f) vel(%.2f, %.2f, %.2f) yaw=%.1f deg",
             seg_idx, t_in_segment, Tseg, px, py, pz, vx, vy, vz, heading 
         );
     }
 };
 
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "traj_gen_continuous");
     TrajGen node;
     ros::spin();
     return 0;
 }
 
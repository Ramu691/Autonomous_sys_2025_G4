/****************************************************
 *  traj_gen.cpp
 *  
 *  Subscribes: /rrt_path (nav_msgs/Path)
 *  Publishes:  /trajectory (MultiDOFJointTrajectoryPoint)
 *
 *  This script generates a multi-segment minimum-snap trajectory.
 *  We use a 7th-order polynomial (8 coefficients) that enforces continuity of position, velocity, and acceleration.
 *  The formulation minimizes snap subject to the boundary conditions.
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
  

 // We compute 8 coefficients for a 7th-order polynomial.
 // The boundary conditions for each segment are chosen as follows:
 // At t=0:  p(0)=p0,  p'(0)=v0,  p''(0)=a0,  p'''(0)=0  (we set jerk = 0)
 // At t=T:  p(T)=pT,  p'(T)=vT,  p''(T)=aT,  p'''(T)=0
 Eigen::VectorXd computeMinSnapCoeffs(double p0, double pT, double T, double v0, double vT, double a0, double aT)
 {
     Eigen::Matrix<double, 8, 8> A;
     A.setZero();
     Eigen::Matrix<double, 8, 1> b;
     b.setZero();
  
     // Boundary conditions at t = 0
     A(0,0) = 1.0;       b(0) = p0;          // p(0) = p0
     A(1,1) = 1.0;       b(1) = v0;          // p'(0) = v0
     A(2,2) = 2.0;       b(2) = a0;          // p''(0) = a0
     A(3,3) = 6.0;       b(3) = 0.0;         // p'''(0) = 0
  
     // Boundary conditions at t = T
     for (int i = 0; i < 8; i++) {
         A(4, i) = std::pow(T, i);    // p(T) = pT
     }
     b(4) = pT;
  
     for (int i = 1; i < 8; i++) {
         A(5, i) = i * std::pow(T, i-1);   // p'(T) = vT
     }
     b(5) = vT;
  
     for (int i = 2; i < 8; i++) {
         A(6, i) = i*(i-1)*std::pow(T, i-2);   // p''(T) = aT
     }
     b(6) = aT;
  
     for (int i = 3; i < 8; i++) {
         A(7, i) = i*(i-1)*(i-2)*std::pow(T, i-3); // p'''(T) = 0
     }
     b(7) = 0.0;
  
     return A.colPivHouseholderQr().solve(b);
 }
  
 // We can use a general polynomial evaluator (works for any order).
 double evalPoly(const Eigen::VectorXd &coeffs, double t, int derivative_order)
 {
     double val = 0.0;
     int n = coeffs.size();
     for (int i = derivative_order; i < n; i++) {
         double factor = 1.0;
         for (int k = 0; k < derivative_order; k++) {
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
  
         ROS_INFO("TrajGen: multi-segment minimum-snap trajectory generator initialized.");
     }
  
 private:
     ros::NodeHandle nh_;
     ros::Subscriber path_sub_, state_sub_, pose_sub_;
     ros::Publisher  desired_pub_;
     ros::Timer      timer_;
      
     geometry_msgs::PoseStamped current_pose_;
     std::string stm_state_;
     bool pose_received_ = false;
  
     // Trajectory variables
     double publish_rate_;
     // For each segment, we now store an 8-element vector (min-snap coefficients) for x, y, z.
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
      * @brief Build the minimum-snap trajectory from the path.
      * 
      * For each segment between consecutive waypoints, we:
      *  - Compute a segment time based on the Euclidean distance.
      *  - Estimate desired velocity and acceleration at each waypoint using finite differences.
      *  - Compute the minimum-snap (7th order) polynomial coefficients for each segment.
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
  
         // Clear previous data.
         cxs_.clear();
         cys_.clear();
         czs_.clear();
         segTimes_.clear();
         cumTimes_.clear();
         total_time_ = 0.0;
  
         // Extract positions.
         std::vector<geometry_msgs::Point> points;
         points.resize(N);
         for (size_t i = 0; i < N; ++i) {
             points[i] = path_msg.poses[i].pose.position;
         }
  
         // Compute segment times based on distance.
         for (size_t i = 0; i < N - 1; ++i) {
             double dx = points[i+1].x - points[i].x;
             double dy = points[i+1].y - points[i].y;
             double dz = points[i+1].z - points[i].z;
             double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
             // Use a speed factor (here 0.15 gives a reasonable duration)
             double Tseg = std::max(1.0, dist * 0.15);
             segTimes_.push_back(Tseg);
         }
  
         // Compute cumulative times.
         cumTimes_.resize(segTimes_.size() + 1, 0.0);
         for (size_t i = 0; i < segTimes_.size(); ++i) {
             cumTimes_[i+1] = cumTimes_[i] + segTimes_[i];
         }
         total_time_ = cumTimes_.back();
  
         // Compute desired velocities and accelerations at waypoints using finite differences.
         std::vector<double> vx(N, 0.0), vy(N, 0.0), vz(N, 0.0);
         std::vector<double> ax(N, 0.0), ay(N, 0.0), az(N, 0.0);
  
         // For the first waypoint, use forward difference.
         vx[0] = (points[1].x - points[0].x) / segTimes_[0];
         vy[0] = (points[1].y - points[0].y) / segTimes_[0];
         vz[0] = (points[1].z - points[0].z) / segTimes_[0];
         // For acceleration, we can simply set to 0 or approximate with a forward difference.
         ax[0] = 0.0; ay[0] = 0.0; az[0] = 0.0;
  
         // For the last waypoint, use backward difference.
         vx[N-1] = (points[N-1].x - points[N-2].x) / segTimes_[N-2];
         vy[N-1] = (points[N-1].y - points[N-2].y) / segTimes_[N-2];
         vz[N-1] = (points[N-1].z - points[N-2].z) / segTimes_[N-2];
         ax[N-1] = 0.0; ay[N-1] = 0.0; az[N-1] = 0.0;
  
         // For intermediate waypoints, use central differences.
         for (size_t i = 1; i < N-1; ++i) {
             double Tprev = segTimes_[i-1];
             double Tnext = segTimes_[i];
             vx[i] = 0.5 * ( (points[i].x - points[i-1].x)/Tprev + (points[i+1].x - points[i].x)/Tnext );
             vy[i] = 0.5 * ( (points[i].y - points[i-1].y)/Tprev + (points[i+1].y - points[i].y)/Tnext );
             vz[i] = 0.5 * ( (points[i].z - points[i-1].z)/Tprev + (points[i+1].z - points[i].z)/Tnext );
             // Approximate acceleration as the change in velocity.
             ax[i] = 0.5 * ((vx[i] - vx[i-1]) / Tprev + (vx[i+1] - vx[i]) / Tnext);
             ay[i] = 0.5 * ((vy[i] - vy[i-1]) / Tprev + (vy[i+1] - vy[i]) / Tnext);
             az[i] = 0.5 * ((vz[i] - vz[i-1]) / Tprev + (vz[i+1] - vz[i]) / Tnext);
         }
  
         // For each segment, compute minimum-snap (7th order) polynomial coefficients for x, y, and z.
         cxs_.resize(N-1);
         cys_.resize(N-1);
         czs_.resize(N-1);
         for (size_t i = 0; i < N - 1; ++i)
         {
             double Tseg = segTimes_[i];
             cxs_[i] = computeMinSnapCoeffs(points[i].x, points[i+1].x, Tseg, vx[i], vx[i+1], ax[i], ax[i+1]);
             cys_[i] = computeMinSnapCoeffs(points[i].y, points[i+1].y, Tseg, vy[i], vy[i+1], ay[i], ay[i+1]);
             czs_[i] = computeMinSnapCoeffs(points[i].z, points[i+1].z, Tseg, vz[i], vz[i+1], az[i], az[i+1]);
         }
  
         has_coeffs_ = true;
         start_time_ = ros::Time::now();
  
         double totalPathDist = computePathDistance(path_msg);
         ROS_INFO("TrajGen: Built %lu segments. Total time = %.2f, Path distance = %.2f",
                  segTimes_.size(), total_time_, totalPathDist);
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
      * @brief Evaluate the minimum-snap polynomial for the given segment and publish the trajectory point.
      */
     void publishPoint(size_t seg_idx, double t_in_segment)
     {
         // Retrieve min-snap coefficients for this segment.
         const Eigen::VectorXd &cx = cxs_[seg_idx];
         const Eigen::VectorXd &cy = cys_[seg_idx];
         const Eigen::VectorXd &cz = czs_[seg_idx];
         double Tseg = segTimes_[seg_idx];
  
         if (t_in_segment > Tseg) t_in_segment = Tseg;
  
         // Evaluate position.
         double px = evalPoly(cx, t_in_segment, 0);
         double py = evalPoly(cy, t_in_segment, 0);
         double pz = evalPoly(cz, t_in_segment, 0);
  
         // Evaluate velocity.
         double vx = evalPoly(cx, t_in_segment, 1);
         double vy = evalPoly(cy, t_in_segment, 1);
         double vz = evalPoly(cz, t_in_segment, 1);
  
         // Evaluate acceleration.
         double ax = evalPoly(cx, t_in_segment, 2);
         double ay = evalPoly(cy, t_in_segment, 2);
         double az = evalPoly(cz, t_in_segment, 2);
  
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
             seg_idx, t_in_segment, Tseg, px, py, pz, vx, vy, vz, heading * 180.0 / M_PI
         );
     }
 };
  
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "traj_gen");
     TrajGen node;
     ros::spin();
     return 0;
 }
 
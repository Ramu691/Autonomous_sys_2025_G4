#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h> //// http://docs.ros.org/en/noetic/api/octomap_msgs/html/msg/Octomap.html
#include <octomap_msgs/conversions.h> //http://docs.ros.org/en/jade/api/octomap_msgs/html/conversions_8h.html
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <custom_msgs/FrontierGoalMsg.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


#define TOL 1



class FrontierDetector{
private:
	struct Point3D {
	    double x, y, z;
	};
	
	struct Frontier{
		Point3D point;
		int count = 0;
		double score = 0;
		bool raytraceable = false;
	};


    ros::NodeHandle nh;
    ros::Subscriber octomap_sub, pos_sub, goal_sub, state_sub;
    ros::Publisher frontier_pub, goal_pub, frontier_goal_pub;
    ros::Timer timer;

    double drone_yaw;
    Point3D drone_pos, goal_pos_sub, cave_entry;
	geometry_msgs::PoseStamped goal_msg;
	custom_msgs::FrontierGoalMsg frontier_goal_msg;	
	float octomap_res;
	std::string stm_state;
	
	
	// score function gains
	double k_dist = 1.0;
	double k_count = 0.1;
	double k_yaw = 30.0;


public:
    FrontierDetector(){
        octomap_sub = nh.subscribe("/octomap_binary", 1, &FrontierDetector::parseOctomap, this);
		timer = nh.createTimer(ros::Duration(1.0), &FrontierDetector::publish_goal,this);
        frontier_pub = nh.advertise<visualization_msgs::MarkerArray>("/frontiers", 1000);
        pos_sub = nh.subscribe("/pose_est",1,&FrontierDetector::onPose,this);
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
        frontier_goal_pub = nh.advertise<custom_msgs::FrontierGoalMsg>("/frontiergoal", 1000);
        // state_sub = nh.subscribe("/Current_State_stm", 1, &FrontierDetector::onStateStm, this);
        
        // cave entry coordinates
		cave_entry.x = -321.0;
		cave_entry.y = 10.0;
		cave_entry.z = 15.0;
		
		goal_msg.header.frame_id = "world";
    }
    
    // void onStateStm(const std_msgs::String& cur_state){
	// 	stm_state = cur_state.data;
	// }
		
    
    void onPose(const geometry_msgs::PoseStamped& msg){
    	drone_pos.x = msg.pose.position.x;
    	drone_pos.y = msg.pose.position.y;
    	drone_pos.z = msg.pose.position.z;
    	
    	tf2::Quaternion quat;
    	tf2::fromMsg(msg.pose.orientation, quat);

		// Convert quaternion to Euler angles
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, drone_yaw);

		// Check if drone has reached the cave entry point
        if (abs(drone_pos.x - cave_entry.x) < 1.0 && abs(drone_pos.y - cave_entry.y) < 1.0 && abs(drone_pos.z - cave_entry.z) < 1.0 && abs(drone_yaw - 3.14) < 0.1) {
            if (stm_state != "Explore Cave") {
                ROS_INFO("Reached cave entry. Starting cave exploration.");
                ros::param::set("/Current_State", "Explore Cave");
				stm_state="Explore Cave";
                resetOctomap();
            }
        }
    }

	void resetOctomap() {
        std_srvs::Empty empty_srv;
        while (!ros::service::call("/octomap_server/reset", empty_srv)) {
            ROS_WARN("Octomap not reset, retrying...");
            ros::Duration(1).sleep();
        }
        ROS_INFO("Octomap reset.");
    }
        
    double euclideanDistance(Point3D p1, Point3D p2) {
		double dx = p1.x - p2.x;
		double dy = p1.y - p2.y;
		double dz = p1.z - p2.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	// Gaussian kernel function
	double gaussianKernel(double distance, double bandwidth) {
		return exp(-(distance * distance) / (2 * bandwidth * bandwidth));
	}

	// Perform mean shift clustering - implemented with help from chatgpt
	std::vector<Point3D> meanShiftClustering(std::vector<Frontier> points, double bandwidth, double convergenceThreshold) {
		std::vector<Point3D> shiftedPoints;
		ROS_INFO("starting meanShiftClustering");
		
		for(Frontier frontier:points){
			shiftedPoints.push_back(frontier.point);
		}

		bool converged = false;
		while (!converged) {
		    converged = true;

		    for (size_t i = 0; i < shiftedPoints.size(); ++i) {
		        Point3D originalPoint = shiftedPoints[i];

		        // Compute the mean shift vector
		        Point3D shiftVector = {0.0, 0.0, 0.0};
		        double totalWeight = 0.0;

		        for (const Frontier& p : points) {
		            double distance = euclideanDistance(originalPoint, p.point);
		            double weight = gaussianKernel(distance, bandwidth);

		            shiftVector.x += p.point.x * weight;
		            shiftVector.y += p.point.y * weight;
		            shiftVector.z += p.point.z * weight;

		            totalWeight += weight;
		        }

		        shiftVector.x /= totalWeight;
		        shiftVector.y /= totalWeight;
		        shiftVector.z /= totalWeight;

		        // Update the current point
		        double distanceToOriginal = euclideanDistance(originalPoint, shiftVector);
		        if (distanceToOriginal > convergenceThreshold) {
		            shiftedPoints[i] = shiftVector;
		            converged = false;
					ROS_WARN("didnot converged !!!!!!!!!!!!!!!!");
		        }
		    }
		}
		ROS_INFO("Clustering result:");
		for (size_t i = 0; i < shiftedPoints.size(); ++i) {
			ROS_INFO("Cluster %zu: x = %f, y = %f, z = %f", i, shiftedPoints[i].x, shiftedPoints[i].y, shiftedPoints[i].z);
		}
		return shiftedPoints;
	}
	
	double get_score(Frontier frontier){		
		double yaw_score = abs(atan2(frontier.point.y-drone_pos.y, frontier.point.x-drone_pos.x)-drone_yaw);
    	if(yaw_score > 3.14){
    		yaw_score = 6.28-yaw_score;
    	}
		ROS_INFO("calculating score");
    	return -k_dist*euclideanDistance(frontier.point, drone_pos) + k_count*(double)frontier.count - k_yaw*yaw_score;
	}
	
	void publish_goal(const ros::TimerEvent& event){
		Point3D goal_point;
		goal_point.x = goal_msg.pose.position.x;
		goal_point.y = goal_msg.pose.position.y;
		goal_point.z = goal_msg.pose.position.z;
		if(stm_state == "Explore Cave"){
	    	goal_pub.publish(goal_msg);
	    	frontier_goal_pub.publish(frontier_goal_msg);
			ROS_INFO(" publishing goal ");
	    }
	}
	
	void set_goal_msg(Frontier best_frontier){
		goal_msg.header.frame_id = "world";
	    goal_msg.pose.position.x = best_frontier.point.x;
	    goal_msg.pose.position.y = best_frontier.point.y;
	    goal_msg.pose.position.z = best_frontier.point.z;
	    double goal_yaw = atan2(best_frontier.point.y-drone_pos.y,best_frontier.point.x-drone_pos.x);
	    tf2::Quaternion q;
		q.setRPY( 0, 0, goal_yaw);
		goal_msg.pose.orientation.x = q.getX();
	    goal_msg.pose.orientation.y = q.getY();
	    goal_msg.pose.orientation.z = q.getZ();
	    goal_msg.pose.orientation.w = q.getW();
		ROS_INFO("setting goal msg  ");
		
	}
	
	void select_frontier(std::vector<Frontier> points_sorted){
		if(points_sorted.size()>0){
		    Frontier best_frontier = points_sorted[0];
		    best_frontier.score = get_score(best_frontier);
		    
		    for(Frontier frontier:points_sorted){
		    	frontier.score = get_score(frontier);
		    	//if(euclideanDistance(f3.point, drone_pos) < euclideanDistance(closest_frontier.point, drone_pos)){
		    	//if(f3.count > closest_frontier.count){
		    	if(frontier.score > best_frontier.score){
		    		best_frontier = frontier;
		    		
		    	}
		    }
		    frontier_goal_msg.point.x = best_frontier.point.x;
	    	frontier_goal_msg.point.y = best_frontier.point.y;
	    	frontier_goal_msg.point.z = best_frontier.point.z;
	    	frontier_goal_msg.raytraceable = best_frontier.raytraceable;
			ROS_INFO("  selecting frontiers");
	    	
			if(best_frontier.raytraceable && euclideanDistance(drone_pos, best_frontier.point) < 40){
		    	set_goal_msg(best_frontier);
		    	
		    }
        }
	}
	
	void publish_markers(std::vector<Frontier> points_sorted){
		visualization_msgs::MarkerArray frontierMarkers;
		
		visualization_msgs::Marker clear_marker;
		clear_marker.id = 0;
		clear_marker.ns = "frontier";
		clear_marker.action = visualization_msgs::Marker::DELETEALL;
		frontierMarkers.markers.push_back(clear_marker);
            
        for(int i=0; i<points_sorted.size();i++){
        	visualization_msgs::Marker marker;
				marker.header.frame_id = "world";
				marker.header.stamp = ros::Time::now();
				marker.ns = "frontier";
				marker.id = i+1;
				marker.type = visualization_msgs::Marker::CUBE;
				
				marker.pose.position.x = points_sorted[i].point.x;
				marker.pose.position.y = points_sorted[i].point.y;
				marker.pose.position.z = points_sorted[i].point.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				marker.scale.x = octomap_res;
				marker.scale.y = octomap_res;
				marker.scale.z = octomap_res;
				
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
				marker.lifetime = ros::Duration();
				frontierMarkers.markers.push_back(marker);

        }
		ROS_INFO("  publishing frontiers");
        frontier_pub.publish(frontierMarkers);
	}

	std::vector<Frontier> sort_frontiers(std::vector<Point3D> frontiers_clustered, octomap::OcTree* octree){
		ROS_INFO(" sorting frontiers ");
		std::vector<Frontier> frontiers_sorted;
		for(Point3D f1:frontiers_clustered){
			Frontier frontier;
			frontier.point = f1;
			frontier.count = 0;
			for(Point3D f2:frontiers_clustered){
				double d = euclideanDistance(f1,f2);
				if(d < octomap_res){
					frontier.count++;
				}
			}
			bool addflag = true;
			for(Frontier f3:frontiers_sorted){
				if(euclideanDistance(f3.point,frontier.point) < octomap_res){
					addflag = false;
				}
			}
			int nbscore = 0;
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dy = -1; dy <= 1; ++dy) {
				    for (int dz = -1; dz <= 1; ++dz) {
				        if (dx != 0 && dy != 0 && dz != 0){
				        	octomap::point3d point(frontier.point.x+octomap_res*dx, frontier.point.y+octomap_res*dy, frontier.point.z+octomap_res*dz);
				        	octomap::OcTreeNode* result = octree->search(point);
				        	if (result != nullptr) {
						    	if (octree->isNodeOccupied(result)) {
						    		nbscore++;
						    	}
						    }	
				        }
				     }
				}
			}
			int entry_tol = 25;
			bool add_entry_frontier = true;
			//std::cout << drone_pos.x << "\t" << frontier.point.x << "\t" << cave_entry.x-entry_tol << "\t" << (drone_pos.x < cave_entry.x-entry_tol && frontier.point.x > cave_entry.x-entry_tol) << "\n";
			if(drone_pos.x < cave_entry.x-entry_tol && frontier.point.x > cave_entry.x-entry_tol) {
				add_entry_frontier = false;
			}
			else{
				add_entry_frontier = true;
			}
			
			
			if(addflag && frontier.count > 120 && add_entry_frontier && nbscore < 3){
				octomap::point3d drone_point(drone_pos.x, drone_pos.y, drone_pos.z);
				octomap::point3d direction(frontier.point.x-drone_pos.x,frontier.point.y-drone_pos.y,frontier.point.z-drone_pos.z);
				double d = euclideanDistance(drone_pos, frontier.point);
				octomap::point3d end;
				frontier.raytraceable = !octree->castRay(drone_point, direction, end, true, d);
				//std::cout << frontier.raytraceable << "\n";
				//std::cout << end << "\n";
				//std::cout << frontier.point.x << "\t" << frontier.point.y << "\t" << frontier.point.z << "\n";
				frontiers_sorted.push_back(frontier);
			}
		}
		ROS_INFO(" frontiers sorted ");
		return frontiers_sorted;
	}
	
	
// http://docs.ros.org/en/noetic/api/octomap_msgs/html/msg/Octomap.html
    void parseOctomap(const octomap_msgs::Octomap::ConstPtr& octomap_msg){ 
		////Convert an octomap representation to a new octree (full probabilities or binary). You will need to free the memory. Return NULL on error. 
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
		//Creates a new octree by deserializing from a message that contains the full map information (i.e., binary is false) and returns
		// an AbstractOcTree* to it. You will need to free the memory when you're done. 
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        std::vector<Frontier> frontiers;
        octomap_res = octree->getResolution();
		ROS_INFO("parsing octmap");
        
        if (octree){
            for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it){
                if (!octree->isNodeOccupied(*it)) {
				    for (int dx = -1; dx <= 1; ++dx) {
				        for (int dy = -1; dy <= 1; ++dy) {
				            for (int dz = -1; dz <= 1; ++dz) {
				                if (dx != 0 && dy != 0 && dz != 0){
				                	octomap::OcTreeKey neighborKey(it.getKey().k[0] + dx, it.getKey().k[1] + dy, it.getKey().k[2] + dz);
				                	if (octree->search(neighborKey)==NULL) {
										Frontier frontierPoint;
		                            	frontierPoint.point.x = it.getX();//+dx;
		                            	frontierPoint.point.y = it.getY();//+dy;
		                            	frontierPoint.point.z = it.getZ();//+dz;
		                            	frontiers.push_back(frontierPoint);
		                       		}
				                }
				            }
				        }
				    }
				}
            }
            
            //std::cout << frontiers.size() << "\n";
            if(stm_state == "Explore Cave"){
		        std::vector<Point3D> frontierpoints_clustered;
		        frontierpoints_clustered = meanShiftClustering(frontiers, 17.0, 1);
		        
		                    
		        std::vector<Frontier> frontierpoints_sorted;
		        frontierpoints_sorted = sort_frontiers(frontierpoints_clustered, octree);
		                    
		        select_frontier(frontierpoints_sorted);
		       	publish_markers(frontierpoints_sorted);
		    }

        }
        else{
            ROS_ERROR("Failed to parse octomap message");
        }
        delete octree;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "frontier_detector_node");
    FrontierDetector frontier_detector;
    ros::spin();
    return 0;
}

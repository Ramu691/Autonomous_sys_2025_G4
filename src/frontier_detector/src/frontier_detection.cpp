#include <ros/ros.h>						// 
#include <octomap/octomap.h>				// 
#include <octomap_msgs/Octomap.h> 			// http://docs.ros.org/en/noetic/api/octomap_msgs/html/msg/Octomap.html
#include <octomap_msgs/conversions.h> 		// http://docs.ros.org/en/jade/api/octomap_msgs/html/conversions_8h.html
#include <visualization_msgs/MarkerArray.h>	// https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html
#include <visualization_msgs/Marker.h>		// https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
#include <geometry_msgs/PoseStamped.h>		// A Pose with reference coordinate frame and timestamp Header header Pose pose
#include <geometry_msgs/Pose.h>				// A representation of pose in free space, composed of position and orientation. Point position Quaternion orientation
#include <geometry_msgs/Point.h> 			// This contains the position of a point in free space float64 x float64 y float64 z
#include <tf/transform_broadcaster.h>		//
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <custom_msgs/FrontierGoalMsg.h>	//
#include <std_msgs/String.h>				//
#include <std_srvs/Empty.h>					//

#define TOL 1

class FrontierDetector{
private:
	struct Point3D {
	    double x, y, z;
	};
	
	struct Frontier{
		Point3D coordinates;
		int neighborcount = 0;
		double score = 0;
		bool isReachable = false;
	};

    ros::NodeHandle nh;
    ros::Subscriber octomap_subscriber, Curr_Pose_Subscriber, goal_subscriber, state_subscriber;
    ros::Publisher frontier_publisher, goal_publisher, frontier_goal_publisher;
    ros::Timer timer;

    double drone_yaw;
    Point3D curr_drone_position, goal_Curr_Pose_Subscriber, cave_entry_point;
	geometry_msgs::PoseStamped goal_message;
	custom_msgs::FrontierGoalMsg frontier_goal_message;	
	float octomap_res;
	std::string statemachine_state;
	
	
	// score function gains to rank the frontiers (Parameter Tunning needed)
	double k_distance = 1.0;
	double k_neighborcount = 0.1;
	double k_yaw = 30.0;


public:
    FrontierDetector(){
        octomap_subscriber = nh.subscribe("/octomap_binary", 1, &FrontierDetector::parseOctomap, this);
		timer = nh.createTimer(ros::Duration(1.0), &FrontierDetector::publish_goal,this);
        frontier_publisher = nh.advertise<visualization_msgs::MarkerArray>("/frontiers", 1000);
        Curr_Pose_Subscriber = nh.subscribe("/pose_est",1,&FrontierDetector::Current_position,this);
        goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
        frontier_goal_publisher = nh.advertise<custom_msgs::FrontierGoalMsg>("/frontier_goal", 1000);
        // state_subscriber = nh.subscribe("/Current_State_stm", 1, &FrontierDetector::onStateStm, this);
        
        // cave entry coordinates
		cave_entry_point.x = -321.0;
		cave_entry_point.y = 10.0;
		cave_entry_point.z = 15.0;
    }
    
    // void onStateStm(const std_msgs::String& cur_state){
	// 	statemachine_state = cur_state.data;
	// }
		
    
    void Current_position(const geometry_msgs::PoseStamped& msg){
    	curr_drone_position.x = msg.pose.position.x;
    	curr_drone_position.y = msg.pose.position.y;
    	curr_drone_position.z = msg.pose.position.z;
    	
    	tf2::Quaternion quat;
    	tf2::fromMsg(msg.pose.orientation, quat);

		// Convert quaternion to Euler angles
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, drone_yaw);

		// Check if drone has reached the cave entry point
        if (abs(curr_drone_position.x - cave_entry_point.x) < 3.0 && abs(curr_drone_position.y - cave_entry_point.y) < 3.0 && abs(curr_drone_position.z - cave_entry_point.z) < 3.0 && abs(drone_yaw - 3.14) < 0.1) {
            if (statemachine_state != "Explore Cave") {
                ROS_INFO("Reached cave entry. Starting cave exploration.");
                ros::param::set("/Current_State", "Explore Cave");
				statemachine_state="Explore Cave";
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
		ROS_INFO("MeanShiftClustering algorithm starting");
		
		for(Frontier frontier:points){
			shiftedPoints.push_back(frontier.coordinates);
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
		            double distance = euclideanDistance(originalPoint, p.coordinates);
		            double weight = gaussianKernel(distance, bandwidth);

		            shiftVector.x += p.coordinates.x * weight;
		            shiftVector.y += p.coordinates.y * weight;
		            shiftVector.z += p.coordinates.z * weight;

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
					//ROS_WARN("didnot converged !!!!!!!!!!!!!!!!");
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
		double yaw_score = abs(atan2(frontier.coordinates.y-curr_drone_position.y, frontier.coordinates.x-curr_drone_position.x)-drone_yaw);
    	if(yaw_score > 3.14){
    		yaw_score = 6.28-yaw_score;
    	}
		ROS_INFO("calculating score");
    	return -k_distance*euclideanDistance(frontier.coordinates, curr_drone_position) + k_neighborcount*(double)frontier.neighborcount - k_yaw*yaw_score;
	}
	
	void publish_goal(const ros::TimerEvent& event){
		Point3D goal_point;
		goal_point.x = goal_message.pose.position.x;
		goal_point.y = goal_message.pose.position.y;
		goal_point.z = goal_message.pose.position.z;
		if(statemachine_state == "Explore Cave"){
	    	goal_publisher.publish(goal_message);
	    	frontier_goal_publisher.publish(frontier_goal_message);
			ROS_INFO(" publishing goal ");
	    }
	}
	
	void set_goal_message(Frontier best_frontier){
		goal_message.header.frame_id = "world";
	    goal_message.pose.position.x = best_frontier.coordinates.x;
	    goal_message.pose.position.y = best_frontier.coordinates.y;
	    goal_message.pose.position.z = best_frontier.coordinates.z;
	    double goal_yaw = atan2(best_frontier.coordinates.y-curr_drone_position.y,best_frontier.coordinates.x-curr_drone_position.x);
	    tf2::Quaternion q;
		q.setRPY( 0, 0, goal_yaw);
		goal_message.pose.orientation.x = q.getX();
	    goal_message.pose.orientation.y = q.getY();
	    goal_message.pose.orientation.z = q.getZ();
	    goal_message.pose.orientation.w = q.getW();

		ROS_INFO("setting goal msg  ");
		
	}
	
	void select_frontier(std::vector<Frontier> points_sorted){
		if(points_sorted.size()>0){
		    Frontier best_frontier = points_sorted[0];
		    best_frontier.score = get_score(best_frontier);		    
		    for(Frontier frontier:points_sorted){
		    	frontier.score = get_score(frontier);		    	
		    	if(frontier.score > best_frontier.score){
		    		best_frontier = frontier;		    		
		    	}
		    }
		    frontier_goal_message.point.x = best_frontier.coordinates.x;
	    	frontier_goal_message.point.y = best_frontier.coordinates.y;
	    	frontier_goal_message.point.z = best_frontier.coordinates.z;
	    	frontier_goal_message.isReachable = best_frontier.isReachable;
			ROS_INFO("  selecting frontiers");
	    	
			if(best_frontier.isReachable && euclideanDistance(curr_drone_position, best_frontier.coordinates) < 40){
		    	set_goal_message(best_frontier);
		    	
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
        	visualization_msgs::Marker new_frontier_marker;
				new_frontier_marker.header.frame_id = "world";
				new_frontier_marker.header.stamp = ros::Time::now();
				new_frontier_marker.ns = "frontier";
				new_frontier_marker.id = i+1;
				new_frontier_marker.type = visualization_msgs::Marker::CUBE;
				
				new_frontier_marker.pose.position.x = points_sorted[i].coordinates.x;
				new_frontier_marker.pose.position.y = points_sorted[i].coordinates.y;
				new_frontier_marker.pose.position.z = points_sorted[i].coordinates.z;
				new_frontier_marker.pose.orientation.x = 0.0;
				new_frontier_marker.pose.orientation.y = 0.0;
				new_frontier_marker.pose.orientation.z = 0.0;
				new_frontier_marker.pose.orientation.w = 1.0;
				
				new_frontier_marker.scale.x = octomap_res;
				new_frontier_marker.scale.y = octomap_res;
				new_frontier_marker.scale.z = octomap_res;
				
				new_frontier_marker.color.r = 1.0f;
				new_frontier_marker.color.g = 0.0f;
				new_frontier_marker.color.b = 0.0f;
				new_frontier_marker.color.a = 1.0;
				new_frontier_marker.lifetime = ros::Duration();
				frontierMarkers.markers.push_back(new_frontier_marker);

        }
		ROS_INFO("  publishing frontiers");
        frontier_publisher.publish(frontierMarkers);
	}

	std::vector<Frontier> sort_frontiers(std::vector<Point3D> frontiers_clustered, octomap::OcTree* octree){
		ROS_INFO(" sorting frontiers ");
		std::vector<Frontier> frontiers_sorted;
		for(Point3D f1:frontiers_clustered){
			Frontier frontier;
			frontier.coordinates = f1;
			frontier.neighborcount = 0;
			for(Point3D f2:frontiers_clustered){
				double d = euclideanDistance(f1,f2);
				if(d < octomap_res){
					frontier.neighborcount++;
				}
			}
			bool addflag = true;
			for(Frontier f3:frontiers_sorted){
				if(euclideanDistance(f3.coordinates,frontier.coordinates) < octomap_res){
					addflag = false;
				}
			}
			int nbscore = 0;
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dy = -1; dy <= 1; ++dy) {
				    for (int dz = -1; dz <= 1; ++dz) {
				        if (dx != 0 && dy != 0 && dz != 0){
				        	octomap::point3d point(frontier.coordinates.x+octomap_res*dx, frontier.coordinates.y+octomap_res*dy, frontier.coordinates.z+octomap_res*dz);
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
			//std::cout << curr_drone_position.x << "\t" << frontier.coordinates.x << "\t" << cave_entry_point.x-entry_tol << "\t" << (curr_drone_position.x < cave_entry_point.x-entry_tol && frontier.coordinates.x > cave_entry_point.x-entry_tol) << "\n";
			if(curr_drone_position.x < cave_entry_point.x-entry_tol && frontier.coordinates.x > cave_entry_point.x-entry_tol) {
				add_entry_frontier = false;
			}
			else{
				add_entry_frontier = true;
			}
			
			
			if(addflag && frontier.neighborcount > 120 && add_entry_frontier && nbscore < 3){
				octomap::point3d drone_point(curr_drone_position.x, curr_drone_position.y, curr_drone_position.z);
				octomap::point3d direction(frontier.coordinates.x-curr_drone_position.x,frontier.coordinates.y-curr_drone_position.y,frontier.coordinates.z-curr_drone_position.z);
				double d = euclideanDistance(curr_drone_position, frontier.coordinates);
				octomap::point3d end;
				frontier.isReachable = !octree->castRay(drone_point, direction, end, true, d);
				//std::cout << frontier.isReachable << "\n";
				//std::cout << end << "\n";
				//std::cout << frontier.coordinates.x << "\t" << frontier.coordinates.y << "\t" << frontier.coordinates.z << "\n";
				frontiers_sorted.push_back(frontier);
			}
		}
		ROS_INFO(" frontiers sorted ");
		return frontiers_sorted;
	}
	
//with partially help of Chatgpt, with the main focus to pase the binary octomap while Exploring cave
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
		                            	frontierPoint.coordinates.x = it.getX();//+dx;
		                            	frontierPoint.coordinates.y = it.getY();//+dy;
		                            	frontierPoint.coordinates.z = it.getZ();//+dz;
		                            	frontiers.push_back(frontierPoint);
		                       		}
				                }
				            }
				        }
				    }
				}
            }
            
            //std::cout << frontiers.size() << "\n";
            if(statemachine_state == "Explore Cave"){
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

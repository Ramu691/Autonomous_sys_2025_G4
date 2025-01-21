#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frontier_detector/FrontierGoalMsg.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PathBuilder{
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
	
	struct PathNode{
		Point3D point;
	};
	
	std::vector<PathNode> graph;


    ros::NodeHandle nh;
    ros::Subscriber frontier_sub;
    ros::Publisher path_pub, marker_pub;
    

public:
    PathBuilder(){
        frontier_sub = nh.subscribe("/frontiergoal",1,&PathBuilder::onFrontier,this);
        path_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/node_markers",10);
    }
    
    double euclideanDistance(Point3D p1, Point3D p2) {
		double dx = p1.x - p2.x;
		double dy = p1.y - p2.y;
		double dz = p1.z - p2.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}
	
	void publish_markers(){
			visualization_msgs::MarkerArray node_markers;
			int i = 0;
			for(PathNode node:graph){
				visualization_msgs::Marker marker;
				marker.header.frame_id = "world";
				marker.header.stamp = ros::Time::now();
				marker.ns = "node";
				marker.id = i;
				marker.type = visualization_msgs::Marker::CUBE;
				
				marker.pose.position.x = node.point.x;
				marker.pose.position.y = node.point.y;
				marker.pose.position.z = node.point.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				marker.scale.x = 1.0;
				marker.scale.y = 1.0;
				marker.scale.z = 1.0;
				
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
				marker.lifetime = ros::Duration();
				
				node_markers.markers.push_back(marker);
				i++;
			}
			marker_pub.publish(node_markers);
		}
    
    void onFrontier(const frontier_detector::FrontierGoalMsg& msg){
    	if(msg.raytraceable){
    		Point3D new_point;
    		new_point.x = msg.point.x;
    		new_point.y = msg.point.y;
    		new_point.z = msg.point.z;
    		bool addflag = true;
    		for(PathNode node:graph){
    			if(euclideanDistance(node.point,new_point) < 1){
    				addflag = false;
    			}
    		}
    		
    		if(addflag){
    			PathNode new_node;
    			new_node.point = new_point;
    			graph.push_back(new_node);
    		}
    	}
    	publish_markers();
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "path_builder_node");
    PathBuilder path_builder;
    ros::spin();
    return 0;
}

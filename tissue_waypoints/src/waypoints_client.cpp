#include<ros/ros.h>
#include<tissue_waypoints/Trajectory.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>


pcl::PointCloud<pcl::PointXYZI> marker_cog_pcl;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, marker_cog_pcl);
	ROS_INFO("got the point cloud data eventually");

}
int main(int argc, char **argv){
	ros::init(argc, argv, "waypoints_client");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	ros::Subscriber pcl_sub = n.subscribe("/nir_overlay_intel/cog",1,get_pcl);
	ros::Publisher traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("tissue_traj",1);
	ros::ServiceClient wp_client = n.serviceClient<tissue_waypoints::Trajectory>("FindTrajectory");
	
	ROS_INFO("service available");
	int seq = 0;
 	while(ros::ok()){
			
		if(marker_cog_pcl.points.size() == 0){
			ROS_INFO("waiting for the pcl to be received");
		}else{
			tissue_waypoints::Trajectory srv;
			srv.request.start.push_back(marker_cog_pcl.points[0].x); 
			srv.request.start.push_back(marker_cog_pcl.points[0].y); 
			srv.request.start.push_back(marker_cog_pcl.points[0].z); 
			srv.request.end.push_back(marker_cog_pcl.points[1].x); 
			srv.request.end.push_back(marker_cog_pcl.points[1].y); 
			srv.request.end.push_back(marker_cog_pcl.points[1].z); 
			if(wp_client.call(srv)){
				ROS_INFO("got the points back:");
				pcl::PointCloud<pcl::PointXYZI> output_traj;
				std::cout << srv.response.path_x.size()<<std::endl;
				for(int i = 0 ; i< srv.response.path_x.size(); i++){
					pcl::PointXYZI xyzi;
					xyzi.x = srv.response.path_x[i];	
					xyzi.y = srv.response.path_y[i];	
					xyzi.z = srv.response.path_z[i];
					xyzi.intensity = 20;	
					output_traj.points.push_back(xyzi);
				}
				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.seq = seq++;
				header.frame_id = std::string("camera_color_optical_frame");
				output_traj.header = pcl_conversions::toPCL(header);
				traj_pub.publish(output_traj);
			}
			else
			{
				ROS_INFO("Failed to call service trajectory");
				return 1;
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

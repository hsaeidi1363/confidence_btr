#include<ros/ros.h>
#include<tissue_waypoints/Trajectory.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>

void detect_key_points(pcl::PointCloud<pcl::PointXYZI> &in_list, pcl::PointCloud<pcl::PointXYZI> & out_traj);

pcl::PointCloud<pcl::PointXYZI> marker_cog_pcl;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, marker_cog_pcl);
	ROS_INFO("got a new reading from marker COG point cloud");

}
//some variables related to the filter using RLS method for fixed positions of the markers
bool filter_initialized = false;
float P = 100.0;
float H = 1.0;
float R = 2.0; //how to set this?
float K = 0.0;
pcl::PointCloud<pcl::PointXYZI> filtered_output_traj;

void filter_waypoints(pcl::PointCloud<pcl::PointXYZI> & _raw, pcl::PointCloud<pcl::PointXYZI> & _filtered){
  if(!filter_initialized){
    for (int i = 0; i < _raw.points.size(); i++){
      _filtered.points.push_back(_raw.points[i]);      
    }
    filter_initialized = true;
  }else{
    K = P*H/(H*P*H + R);
    P *= (1 - K*H);
	ROS_INFO("P is: %f and K is: %f", P,K);
    for (int i = 0; i < _raw.points.size();i++){
      _filtered.points[i].x += K*(_raw.points[i].x-H*_filtered.points[i].x);    
      _filtered.points[i].y += K*(_raw.points[i].y-H*_filtered.points[i].y);    
      _filtered.points[i].z += K*(_raw.points[i].z-H*_filtered.points[i].z);    
	  _filtered.points[i].intensity = 50;
    }
  }  
}

int main(int argc, char **argv){
	ros::init(argc, argv, "waypoints_client");
	ros::NodeHandle n;
	ros::Rate loop_rate(2);
	ros::Subscriber pcl_sub = n.subscribe("/nir_overlay_intel/cog",1,get_pcl);
	ros::Publisher traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("tissue_traj",1);
	ros::Publisher short_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("short_tissue_traj",1);
	ros::Publisher filtered_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("filtered_tissue_traj",1);
	ros::ServiceClient wp_client = n.serviceClient<tissue_waypoints::Trajectory>("FindTrajectory");
	
	ROS_INFO("service available");
	int seq = 0;
 	while(ros::ok()){

		pcl::PointCloud<pcl::PointXYZI> output_traj;			
		pcl::PointCloud<pcl::PointXYZI> short_output_traj;		

		if(marker_cog_pcl.points.size() == 0){
			ROS_INFO("waiting for the pcl to be received");
		}else{
			int pts_len = marker_cog_pcl.points.size();
			for (int pts_id =0; pts_id < pts_len; pts_id++){
				tissue_waypoints::Trajectory srv;
				int start_pt = pts_id % pts_len;
				int end_pt = (pts_id +1 )% pts_len;
				srv.request.start.push_back(marker_cog_pcl.points[start_pt].x); 
				srv.request.start.push_back(marker_cog_pcl.points[start_pt].y); 
				srv.request.start.push_back(marker_cog_pcl.points[start_pt].z); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].x); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].y); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].z); 
				if(wp_client.call(srv)){
					ROS_INFO("got the points back:");

					std::cout << srv.response.path_x.size()<<std::endl;
					pcl::PointCloud<pcl::PointXYZI> pt_list;
					for(int i = 0 ; i< srv.response.path_x.size(); i++){
						pcl::PointXYZI xyzi;
						xyzi.x = srv.response.path_x[i];	
						xyzi.y = srv.response.path_y[i];	
						xyzi.z = srv.response.path_z[i];
						xyzi.intensity = 100;	
						output_traj.points.push_back(xyzi);
						pt_list.points.push_back(xyzi);
					}
					detect_key_points(pt_list,short_output_traj);
					std_msgs::Header header;
					header.stamp = ros::Time::now();
					header.seq = seq++; // is this correct?
					header.frame_id = std::string("camera_color_optical_frame");
					output_traj.header = pcl_conversions::toPCL(header);
					short_output_traj.header = pcl_conversions::toPCL(header);
					traj_pub.publish(output_traj);
					short_traj_pub.publish(short_output_traj);
				}
				else
				{
					ROS_INFO("Failed to call service trajectory");
					return 1;
				}
			}
      		filter_waypoints(short_output_traj,filtered_output_traj);
			ROS_INFO("Filtered one loop of traj");
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.frame_id = std::string("camera_color_optical_frame");
			filtered_output_traj.header = pcl_conversions::toPCL(header);
     		filtered_traj_pub.publish(filtered_output_traj);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

float dist_3d(pcl::PointXYZI & pt1, pcl::PointXYZI & pt2){
	float dx = pt1.x - pt2.x;
	float dy = pt1.y - pt2.y;
	float dz = pt1.z - pt2.z;
	return std::sqrt( dx*dx +dy*dy + dz*dz);
}
void detect_key_points(pcl::PointCloud<pcl::PointXYZI> &in_list, pcl::PointCloud<pcl::PointXYZI> & out_traj){
	float traj_length = 0.0;
	int mid_points = 3;
	std::vector<float> points_loc;// the location of point on the path based on the traveled distance between the start and end point
	points_loc.push_back(0.0); //distance of the start point on the path is zero
	for (int i = 1; i < in_list.points.size(); i++){
		traj_length += dist_3d(in_list.points[i], in_list.points[i-1]);
		points_loc.push_back(traj_length);
	} 
	float traj_seg_len = (float)traj_length/ (mid_points+1);
	for (int j = 1; j<= mid_points; j++){
		float min_dist = 1000.0;
		int min_ind = 0;
		float mid_point_loc = traj_seg_len*j; // e.g. for mid_points = 4 (4 waypoints between the start and end) we will have 5 path segments and mid_point_loc = 20%, 40%, 60%, and 80% of the path
		for (int i = 0; i < in_list.points.size(); i++){
			if (fabs(mid_point_loc - points_loc[i]) < min_dist){
				min_dist = fabs(mid_point_loc - points_loc[i]);
				min_ind = i;
			}
		}
		out_traj.points.push_back(in_list.points[min_ind]);
	}
}

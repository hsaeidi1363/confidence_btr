#include<ros/ros.h>
#include<tissue_waypoints/Trajectory.h>
#include<sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>

void detect_key_points(pcl::PointCloud<pcl::PointXYZI> &in_list, pcl::PointCloud<pcl::PointXYZI> & out_traj);

geometry_msgs::Polygon markers3D;
void get_polygon(const geometry_msgs::Polygon & _data){
	markers3D = _data;	
	ROS_INFO("got a new reading from 3D polygons ");
}

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
	ros::init(argc, argv, "waypoints_linear");
	ros::NodeHandle n;
	ros::Rate loop_rate(2);
	ros::Subscriber pcl_sub = n.subscribe("/nir_overlay_intel/cog",1,get_pcl);
	ros::Subscriber polygon_sub = n.subscribe("/nir_overlay_intel/polygon3D_cog",1, get_polygon);
	ros::Publisher short_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("short_tissue_traj",1);
	ros::Publisher filtered_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("filtered_tissue_traj",1);

	
	int seq = 0;
 	while(ros::ok()){
	
		pcl::PointCloud<pcl::PointXYZI> short_output_traj;		

		if(marker_cog_pcl.points.size() == 0 || marker_cog_pcl.points.size() == 1){
			ROS_INFO("waiting for the pcl points (min = 2) to be received");
		}else{
			int pts_len = marker_cog_pcl.points.size();
			for (int pts_id =0; pts_id < pts_len; pts_id++){
				int start_pt = pts_id % pts_len;
				int end_pt = (pts_id +1 )% pts_len;
				pcl::PointXYZI start_point;
				pcl::PointXYZI end_point;
				pcl::PointCloud<pcl::PointXYZI> pt_list;

				start_point.x = markers3D.points[start_pt].x; 
				start_point.y = markers3D.points[start_pt].y; 
				start_point.z = markers3D.points[start_pt].z; 
				start_point.intensity = 100;
				
				pt_list.points.push_back(start_point);

				end_point.x = markers3D.points[end_pt].x; 
				end_point.y = markers3D.points[end_pt].y; 
				end_point.z = markers3D.points[end_pt].z; 
				end_point.intensity = 100;

				pt_list.points.push_back(end_point);
				detect_key_points(pt_list,short_output_traj);
				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.seq = seq++; // is this correct?
				header.frame_id = std::string("camera_color_optical_frame");
				short_output_traj.header = pcl_conversions::toPCL(header);
				short_traj_pub.publish(short_output_traj);
				
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
	int mid_points = 4;
	
	pcl::PointXYZI p1 = in_list[0];
	pcl::PointXYZI p2 = in_list[1];
        float x1 = p1.x;
	float x2 = p2.x;
      
	float y1 = p1.y;
	float y2 = p2.y;

	float z1 = p1.z;
	float z2 = p2.z;

        float dx=(x2-x1)/(mid_points+1);
        float dy=(y2-y1)/(mid_points+1);
        float dz=(z2-z1)/(mid_points+1);


	for (int j = 1; j<= (mid_points + 1); j++){
		 // e.g. for mid_points = 4 (4 waypoints between the start and end) we will have 5 path segments and mid_point_loc = 20%, 40%, 60%, and 80% of the path
		pcl::PointXYZI pi;
		pi.x = x1 + dx*(j);
		pi.y = y1 + dy*(j);
		pi.z = z1 + dz*(j);
		pi.intensity = 100;
		out_traj.points.push_back(pi);
	}
}


      
     


	

   
	

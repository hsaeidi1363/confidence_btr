#include<ros/ros.h>
#include<tissue_waypoints/Trajectory.h>
#include<sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<std_msgs/Bool.h>

bool freeze_path = false;

void get_freeze(const std_msgs::Bool & _data){
	freeze_path = _data.data;	
}


void detect_key_points(pcl::PointCloud<pcl::PointXYZI> &in_list, pcl::PointCloud<pcl::PointXYZI> & out_traj);



geometry_msgs::Polygon tumor3D;
geometry_msgs::Polygon tumor3D_old;


void get_polygon(const geometry_msgs::Polygon & _data){
	tumor3D = _data;	
	ROS_INFO("got a new reading from 3D polygons ");
}

pcl::PointCloud<pcl::PointXYZI> tumor_pcl;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, tumor_pcl);
	ROS_INFO("got a new reading from tumor point cloud");

}
//some variables related to the filter using RLS method for fixed positions of the markers
bool filter_initialized = false;
float P = 100.0;
float H = 1.0;
float R = 2.0; //how to set this?
float K = 0.0;

pcl::PointCloud<pcl::PointXYZI> filtered_output_traj;
pcl::PointCloud<pcl::PointXYZI> filtered_output_traj_old;

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
	ros::Subscriber freeze_path_sub = n.subscribe("/freeze_path",1,get_freeze);	
	ros::Subscriber pcl_sub = n.subscribe("/nir_overlay_intel_tumor/tumor",1,get_pcl);
	ros::Subscriber polygon_sub = n.subscribe("/nir_overlay_intel_tumor/polygon3D_tumor",1, get_polygon);

	ros::Publisher short_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("short_tissue_traj",1);
	ros::Publisher filtered_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("filtered_tissue_traj",1);


	
	int seq = 0;
	int seq2 = 0;
 	while(ros::ok()){
	
		pcl::PointCloud<pcl::PointXYZI> short_output_traj;	


		if(tumor_pcl.points.size() == 0 || tumor_pcl.points.size() == 1){
			ROS_INFO("waiting for the pcl points (min = 2) to be received");
		}else{
			
			int pts_len = tumor_pcl.points.size();
			ROS_INFO("tumor pcl has %d points",pts_len);
			/*int start_pt = 0;
			int end_pt = pts_len - 1;
			pcl::PointXYZI start_point;
			pcl::PointXYZI end_point;
			pcl::PointCloud<pcl::PointXYZI> pt_list;

			start_point.x = tumor3D.points[start_pt].x; 
			start_point.y = tumor3D.points[start_pt].y; 
			start_point.z = tumor3D.points[start_pt].z; 
			start_point.intensity = 100;
			
			pt_list.points.push_back(start_point);

			end_point.x = tumor3D.points[end_pt].x; 
			end_point.y = tumor3D.points[end_pt].y; 
			end_point.z = tumor3D.points[end_pt].z; 
			end_point.intensity = 100;
			

			pt_list.points.push_back(end_point);
			detect_key_points(pt_list,short_output_traj);

			*/			
			//detect_key_points(tumor_pcl,short_output_traj);
			short_output_traj = tumor_pcl;

			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++; // is this correct?
			header.frame_id = std::string("camera_color_optical_frame");
			short_output_traj.header = pcl_conversions::toPCL(header);
			short_traj_pub.publish(short_output_traj);
				
			if(freeze_path){
				filtered_output_traj = filtered_output_traj_old;
			}else{
	      			filter_waypoints(short_output_traj,filtered_output_traj);
			}
			filtered_output_traj_old = filtered_output_traj;
			ROS_INFO("Filtered one loop of traj");

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
	int mid_points = 12;
	std::vector<float> points_loc;// the location of point on the path based on the traveled distance between the start and end point
	points_loc.push_back(0.0); //distance of the start point on the path is zero
	for (int i = 1; i < in_list.points.size(); i++){
		traj_length += dist_3d(in_list.points[i], in_list.points[i-1]);
		points_loc.push_back(traj_length);
	} 
	float traj_seg_len = (float)traj_length/ (mid_points+1);
	for (int j = 1; j<= (mid_points + 1); j++){
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


      



	

   
	

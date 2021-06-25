#include<ros/ros.h>
#include<tissue_waypoints/Trajectory.h>
#include<sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Point32.h>

bool freeze_path = false;
bool cropped_pcl_available = false;

void get_freeze(const std_msgs::Bool & _data){
	freeze_path = _data.data;	
}

void detect_key_points(pcl::PointCloud<pcl::PointXYZI> &in_list, pcl::PointCloud<pcl::PointXYZI> & out_traj);

geometry_msgs::Polygon add_offset(geometry_msgs::Polygon & polygon);

geometry_msgs::Polygon markers3D;
geometry_msgs::Polygon markers3D_old;
geometry_msgs::Polygon markers3D_offset;

void get_polygon(const geometry_msgs::Polygon & _data){
	markers3D = _data;	
	ROS_INFO("got a new reading from 3D polygons ");
}

pcl::PointCloud<pcl::PointXYZI> marker_cog_pcl;
pcl::PointCloud<pcl::PointXYZI> cropped_pcl;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, marker_cog_pcl);
	ROS_INFO("got a new reading from marker COG point cloud");

}


void get_cropped_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, cropped_pcl);
	ROS_INFO("got a CROPPED point cloud from passthrough filter");
	cropped_pcl_available = true;


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
	ros::init(argc, argv, "waypoints_client");
	ros::NodeHandle n;
	ros::Rate loop_rate(2);
	ros::Subscriber freeze_path_sub = n.subscribe("/freeze_path",1,get_freeze);	
	ros::Subscriber pcl_sub = n.subscribe("/nir_overlay_intel/cog",1,get_pcl);
	ros::Subscriber cropped_pcl_sub = n.subscribe("/d415/passthrough",1,get_cropped_pcl);

	ros::Subscriber polygon_sub = n.subscribe("/nir_overlay_intel/polygon3D_cog",1, get_polygon);
	ros::Publisher traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("tissue_traj",1);
	ros::Publisher short_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("short_tissue_traj",1);
	ros::Publisher filtered_traj_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("filtered_tissue_traj",1);
	ros::Publisher offset_cog_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/nir_overlay_intel/cog_offset",1);
	ros::ServiceClient wp_client = n.serviceClient<tissue_waypoints::Trajectory>("FindTrajectory");
	
	ROS_INFO("service available");
	int seq = 0;
	int seq2 = 0;
 	while(ros::ok()){

		pcl::PointCloud<pcl::PointXYZI> output_traj;			
		pcl::PointCloud<pcl::PointXYZI> short_output_traj;
		pcl::PointCloud<pcl::PointXYZI> offset_cog;		

		if(marker_cog_pcl.points.size() == 0 || marker_cog_pcl.points.size() == 1){
			ROS_INFO("waiting for the pcl points (min = 2) to be received");
		}else{

			//if(freeze_path)
			//	markers3D = markers3D_old; //use the last set of markers if we need to temporarily freeze the path otherwise use the most recent one
			int pts_len = marker_cog_pcl.points.size();
			if (pts_len > 2){
				markers3D_offset = add_offset(markers3D);
				ROS_INFO("added offset to polygon");
				offset_cog.points.clear();	
				for(int off_ind = 0; off_ind < markers3D_offset.points.size(); off_ind ++){
					pcl::PointXYZI xyzi;
					xyzi.x = markers3D_offset.points[off_ind].x;	
					xyzi.y = markers3D_offset.points[off_ind].y;
					xyzi.z = markers3D_offset.points[off_ind].z;
					xyzi.intensity = marker_cog_pcl.points[off_ind].intensity;//?	
					offset_cog.points.push_back(xyzi);
				}
				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.seq = seq2++; 
				header.frame_id = std::string("camera_depth_optical_frame");
				offset_cog.header = pcl_conversions::toPCL(header);
				offset_cog_pub.publish(offset_cog);
			}
			else{
				markers3D_offset = markers3D; // just use the same points without offset
			}
			for (int pts_id =0; pts_id < pts_len; pts_id++){
				tissue_waypoints::Trajectory srv;
				int start_pt = pts_id % pts_len;
				int end_pt = (pts_id +1 )% pts_len;
				/*srv.request.start.push_back(marker_cog_pcl.points[start_pt].x); 
				srv.request.start.push_back(marker_cog_pcl.points[start_pt].y); 
				srv.request.start.push_back(marker_cog_pcl.points[start_pt].z); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].x); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].y); 
				srv.request.end.push_back(marker_cog_pcl.points[end_pt].z); 
				*/
				srv.request.start.push_back(markers3D_offset.points[start_pt].x); 
				srv.request.start.push_back(markers3D_offset.points[start_pt].y); 
				srv.request.start.push_back(markers3D_offset.points[start_pt].z); 
				srv.request.end.push_back(markers3D_offset.points[end_pt].x); 
				srv.request.end.push_back(markers3D_offset.points[end_pt].y); 
				srv.request.end.push_back(markers3D_offset.points[end_pt].z); 				
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
					//pt_list.points.push_back(marker_cog_pcl.points[end_pt]);
					pcl::PointXYZI xyzi;
					xyzi.x = markers3D_offset.points[end_pt].x;	
					xyzi.y = markers3D_offset.points[end_pt].y;	
					xyzi.z = markers3D_offset.points[end_pt].z;
					xyzi.intensity = 100;	
					pt_list.points.push_back(xyzi);
					detect_key_points(pt_list,short_output_traj);
					std_msgs::Header header;
					header.stamp = ros::Time::now();
					header.seq = seq++; // is this correct?
					header.frame_id = std::string("camera_depth_optical_frame");
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
			if(freeze_path){
				filtered_output_traj = filtered_output_traj_old;
			}else{
	      			filter_waypoints(short_output_traj,filtered_output_traj);
			}
			filtered_output_traj_old = filtered_output_traj;
			ROS_INFO("----------Filtered loop %d of traj",seq2);
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.frame_id = std::string("camera_depth_optical_frame");
			filtered_output_traj.header = pcl_conversions::toPCL(header);
	 		filtered_traj_pub.publish(filtered_output_traj);
		}
		markers3D_old = markers3D;
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
	int mid_points = 5;
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


geometry_msgs::Vector3 find_unit_vector(geometry_msgs::Point32 & pt_tail, geometry_msgs::Point32 & pt_head){
	float dx = pt_head.x - pt_tail.x;
	float dy = pt_head.y - pt_tail.y;
	float dz = pt_head.z - pt_tail.z;
	float vec_len =  std::sqrt( dx*dx +dy*dy + dz*dz);
	geometry_msgs::Vector3 unit_vec;
	// make sure the vector length is not close to 0
	if (vec_len > 0.0001){
		unit_vec.x = dx/vec_len; 
		unit_vec.y = dy/vec_len; 
		unit_vec.z = dz/vec_len; 
	}//otherwise just return zero
	return unit_vec;
}


geometry_msgs::Polygon add_offset(geometry_msgs::Polygon & polygon){
	int len = markers3D.points.size();
	float offset_val = 0.005; //offset in METERS
	geometry_msgs::Polygon polygon_offset;
	for (int i = 0; i < len; i++){
		int cur_pt = i ;
		int front_pt = (i + len +1 )% len;
		int back_pt = (i + len - 1 )% len;
		geometry_msgs::Vector3 v1 = find_unit_vector(polygon.points[back_pt], polygon.points[cur_pt]);
		geometry_msgs::Vector3 v2 = find_unit_vector(polygon.points[front_pt], polygon.points[cur_pt]);
		geometry_msgs::Point32 point_offset;
		point_offset.x = polygon.points[cur_pt].x + (v1.x + v2.x)*offset_val;
		point_offset.y = polygon.points[cur_pt].y + (v1.y + v2.y)*offset_val;
		point_offset.z = polygon.points[cur_pt].z + (v1.z + v2.z)*offset_val;
		if(cropped_pcl_available){
			pcl::PointCloud<pcl::PointXYZI>::iterator pi = cropped_pcl.begin();
			pcl::PointCloud<pcl::PointXYZI>::iterator pi_min = cropped_pcl.begin();
			pcl::PointXYZI xyzi;
			xyzi.x = point_offset.x;	
			xyzi.y = point_offset.y;	
			xyzi.z = point_offset.z;
			xyzi.intensity = 100;
			float min_dist = 100;
			for( ; pi!=cropped_pcl.end(); pi++ ) {
				pcl::PointXYZI xyzi2;
				xyzi2.x = pi->x;	
				xyzi2.y = pi->y;	
				xyzi2.z = pi->z;
				xyzi2.intensity = 100;
				float dist_tmp = dist_3d(xyzi,xyzi2);
				if (dist_tmp < min_dist){
					min_dist = dist_tmp;
					pi_min = pi;
				}

			}
			point_offset.x = pi_min->x;
			point_offset.y = pi_min->y;
			point_offset.z = pi_min->z;
		}
		polygon_offset.points.push_back(point_offset);
	}	

	
	return polygon_offset;

}


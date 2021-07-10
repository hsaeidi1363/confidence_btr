#include"confidence_functions.h"
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/transforms.h>
#include<pcl_conversions/pcl_conversions.h>
#include<std_msgs/Float32.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI> cropped_pcl;
pcl::PointCloud<pcl::PointXYZI> path_pcl;


void get_cropped_pcl(const sensor_msgs::PointCloud2Ptr & cloud){
  pcl::fromROSMsg(*cloud, cropped_pcl);
  ROS_INFO("got a CROPPED point cloud from the passthrough filter");


}
void get_path(const sensor_msgs::PointCloud2Ptr& cloud){
    pcl::fromROSMsg(*cloud, path_pcl);
    ROS_INFO("got a filtered path");
}




geometry_msgs::Polygon markers3D;

void get_polygon(const geometry_msgs::Polygon & _data){
	markers3D = _data;	
	ROS_INFO("got a new reading from 3D polygons ");
}


float confidence_threshold = 8.4241;
void get_threshold(const std_msgs::Float32 & _data){
	confidence_threshold = _data.data;	
	std::cout <<"------------------------------- " << std::endl;
	std::cout << "Threshold changed to " << confidence_threshold << std::endl;
	std::cout <<"------------------------------- " << std::endl;
}

float calc_projected_err(float roll_, float pitch_, float dist_, float density_){
	float w0 = 1.0;//0.65; //for overall normalization based on observed errors	
	float w1 = 1.0;//0.25;//weight for roll
	float w2 = 1.0;//0.25;//weight for pitch
	float w3 = 1.0;//0.5;//weight for dist
	float w4 = 1.0;//weight for density

	float e1 = 0.000065*roll_*roll_ + 0.0046*roll_ + 1.2134;
	float e2 = 0.0001564*pitch_*pitch_ - 0.00016*pitch_ + 1.1737;
	float e3 = 0.0761*(dist_*100-11) - 1.3919;//converted to cm first
	float e4 =  10.5458*exp(-0.8991*density_);

	std::cout<<" e1: "<< e1 <<" e2: "<< e2 <<" e3: "<< e3 <<" e4: "<< e4 <<std::endl; 
	return w0*(w1*e1+w2*e2+w3*e3+w4*e4);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "confidence_modles");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(1);


  ros::Subscriber cropped_pcl_sub = nh_.subscribe("/d415/passthrough", 1, get_cropped_pcl);
  ros::Subscriber path_sub = nh_.subscribe("/filtered_tissue_traj",1,get_path);
  ros::Subscriber polygon_sub = nh_.subscribe("/nir_overlay_intel/polygon3D_cog",1, get_polygon);
  ros::Subscriber switch_threshold_sub = nh_.subscribe("/confidence_threshold",1, get_threshold);

  ros::Publisher pub_pcl= nh_.advertise<sensor_msgs::PointCloud2>( "/planefit_dbg", 10 );
  ros::Publisher pub_path_confidence= nh_.advertise<sensor_msgs::PointCloud2>( "/path_confidence", 10 );
  ros::Publisher dbg_pcl= nh_.advertise<geometry_msgs::Twist>( "/density_dbg", 10 );

  geometry_msgs::Vector3 x0;
  geometry_msgs::Vector3 x1;
  geometry_msgs::Vector3 x2;

  
  x0.x = 5.0;
  x0.y = 0.0;
  x0.z = 0.0;


  x1.x = 0.0;
  x1.y = 0.0;
  x1.z = 1.0;
 
  x2.x = 0.0;
  x2.y = 1.0;
  x2.z = 1.0;

  float distance = 0.0;
  distance = dist_from_line(x0, x1, x2);
  std::cout<< distance<<std::endl;

  bool point_check = false;
  
  if (distance < 2){
    point_check = true;
  }else{
    point_check - false;
  }
 
  cout << point_check<<endl;

  int seq = 1;
  while(ros::ok()){

/*		int pts_len = markers3D.points.size();
		if(pts_len == 0 || pts_len == 1){
			ROS_INFO("waiting for the pcl points (min = 2) to be received");
		}else{
			for (int pts_id =0; pts_id < pts_len; pts_id++){
				int start_pt = pts_id % pts_len;
				int end_pt = (pts_id +1 )% pts_len;

				x1.x = markers3D.points[start_pt].x;
				x1.y = markers3D.points[start_pt].y;
				x1.z = markers3D.points[start_pt].z;

				x2.x = markers3D.points[end_pt].x;
				x2.y = markers3D.points[end_pt].y;
				x2.z = markers3D.points[end_pt].z;

				float density = calc_density(x1, x2, cropped_pcl);
				float noise = calc_noise(x1, x2, cropped_pcl);
				std::cout<<"between points "<< start_pt<< " and " << end_pt<< " density is: "<< density << " and noise is: "<<noise<<std::endl;
//				std::cout<<"noise between points "<< start_pt<< " and " << end_pt<< " is: "<< noise<<std::endl;

			}

		}
*/

		//plane parameters
		float a = 0.0;
		float b = 0.0;
		float c = -1.0;
		float d = 0.0;
		fit_plane(a, b, d, cropped_pcl);
		std::cout<<" the plane parameters are: "<< a << ", "<< b << ", "<< c<< ", "<< d<< std::endl;
		// checking the distance of the camera origing to the fitted plane
		float e = sqrt(a*a + b*b + c*c);
		float f = fabs(d)/e; // since the origin has (0,0,0)=> ax+by+z+d is equal to d
		//http://www.nabla.hr/CG-LinesPlanesIn3DA3.htm
		// angular position of the fitted plane compared to the x axis of camera
		float pitch = 90-acos(fabs(a/e))*180/3.1415;
		// angular position of the fitted plane compared to the y axis of camera		
		float roll = -(90 - acos(fabs(b/e))*180/3.1415);
		std::cout<<" distance from origin is: " << f<< std::endl;
		std::cout<<" rotation angle around X is: " << roll<<" rotation angle around Y is: " << pitch<< std::endl;

		pcl::PointCloud<pcl::PointXYZI> confidence_path_pcl;
		int pts_len = path_pcl.points.size();
		if(pts_len == 0 || pts_len == 1){
			ROS_INFO("waiting for the pcl points (min = 2) to be received");
		}else{
			for (int pts_id =0; pts_id < pts_len; pts_id++){
				int start_pt = pts_id % pts_len;
				int end_pt = (pts_id +1 )% pts_len;

				x1.x = path_pcl.points[start_pt].x;
				x1.y = path_pcl.points[start_pt].y;
				x1.z = path_pcl.points[start_pt].z;

				x2.x = path_pcl.points[end_pt].x;
				x2.y = path_pcl.points[end_pt].y;
				x2.z = path_pcl.points[end_pt].z;

				float density = calc_density(x1, x2, cropped_pcl);
				float noise = calc_noise(x1, x2, cropped_pcl);
				float mid_dist = mid_point_dist(x1, x2);
				std::cout<<"between points "<< start_pt<< " and " << end_pt<< " density is: "<< density << " and noise is: "<<noise<<std::endl;
				std::cout<<"and dist to origin is: "<<mid_dist<<std::endl;
//				std::cout<<"noise between points "<< start_pt<< " and " << end_pt<< " is: "<< noise<<std::endl;
				pcl::PointXYZI path_pt;
				path_pt.x = path_pcl.points[start_pt].x;
				path_pt.y = path_pcl.points[start_pt].y;
				path_pt.z = path_pcl.points[start_pt].z;
				float proj_err = calc_projected_err(roll, pitch, mid_dist, density);
				std::cout<<"projected error is: "<< proj_err<<std::endl;
				if(proj_err > confidence_threshold)
				//if(density < 3.0)
					path_pt.intensity = 1;
				else
					path_pt.intensity = 0;

				confidence_path_pcl.points.push_back(path_pt);
			}

		}
				
	
		std::cout<<" end of one round"<<std::endl;

		pcl::PointCloud<pcl::PointXYZI> pclplane;

		show_plane(a,b,d, pclplane);
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.seq = seq++;
		header.frame_id = std::string( "/camera_depth_optical_frame" );
		
		pclplane.header = pcl_conversions::toPCL( header );
		pub_pcl.publish( pclplane );

		confidence_path_pcl.header = pcl_conversions::toPCL( header );
		pub_path_confidence.publish( confidence_path_pcl );

		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;

}

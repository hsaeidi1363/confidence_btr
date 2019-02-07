#include"confidence_functions.h"
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/Polygon.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/transforms.h>
#include<pcl_conversions/pcl_conversions.h>


using namespace std;

pcl::PointCloud<pcl::PointXYZI> cropped_pcl;


void get_cropped_pcl(const sensor_msgs::PointCloud2Ptr & cloud){
  pcl::fromROSMsg(*cloud, cropped_pcl);
  ROS_INFO("got a CROPPED point cloud from the passthrough filter");


}



geometry_msgs::Polygon markers3D;

void get_polygon(const geometry_msgs::Polygon & _data){
	markers3D = _data;	
	ROS_INFO("got a new reading from 3D polygons ");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "confidence_modles");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(1);


  ros::Subscriber cropped_pcl_sub = nh_.subscribe("/d415/passthrough", 1, get_cropped_pcl);
  ros::Subscriber polygon_sub = nh_.subscribe("/nir_overlay_intel/polygon3D_cog",1, get_polygon);

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


  while(ros::ok()){

		int pts_len = markers3D.points.size();
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
				std::cout<<"density between points"<< start_pt<< " and " << end_pt<< " is: "<< density<<std::endl;

			}

		}
		std::cout<<" end of one round"<<std::endl;

		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;

}

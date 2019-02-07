#include"confidence_functions.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami
#include<geometry_msgs/Vector3.h>



using namespace std;


geometry_msgs::Vector3 cross_product(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;

  c.x = a_.y*b_.z - a_.z*b_.y;
  c.y = a_.z*b_.x - a_.x*b_.z;
  c.z = a_.x*b_.y - a_.y*b_.x;

  return c;
}


geometry_msgs::Vector3 subt(geometry_msgs::Vector3 & a_ , geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;
  c.x = a_.x - b_.x;
  c.y = a_.y - b_.y;
  c.z = a_.z - b_.z;

  return c;
}

float norm(geometry_msgs::Vector3 & a_){

  return sqrt(a_.x*a_.x + a_.y*a_.y + a_.z*a_.z);

}


float dist_from_line(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2){

  geometry_msgs::Vector3 diff1;  
  geometry_msgs::Vector3 diff2;
  geometry_msgs::Vector3 diff3;

  diff1 = subt(x0, x1);  
  diff2 = subt(x0, x2);  
  diff3 = subt(x2, x1);  

  geometry_msgs::Vector3 cross1;
  
  cross1 = cross_product(diff1, diff2);
  


  float d = 0.0;
  d = norm(cross1)/norm(diff3);
  return d;

}


float calc_density(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd){
	pcl::PointCloud<pcl::PointXYZI>::iterator pi=pcd.begin();
	float density = 0.0;
	int ctr = 0;
	for( ; pi!=pcd.end(); pi++ ) {

		geometry_msgs::Vector3 x0;
		x0.x = pi->x;
		x0.y = pi->y;
		x0.z = pi->z;
	
		float distance = 0.0;
		distance = dist_from_line(x0, x1, x2);
  
		if (distance <= 0.005)
			density += 1.0;
 		ctr ++;
	}

	return density/pcd.points.size();
}

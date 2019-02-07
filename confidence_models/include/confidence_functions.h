#ifndef CONFIDENCE_FUNCTIONS_
#define CONFIDENCE_FUNCTIONS_


#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami

#include<geometry_msgs/Vector3.h>

geometry_msgs::Vector3 cross_product(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_);


geometry_msgs::Vector3 subt(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_);


float norm(geometry_msgs::Vector3 & a_);

// calculate the distance of point x0 to the line formed by points x1 and x2
float dist_from_line(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1,geometry_msgs::Vector3 x2);


float calc_density(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd);

#endif

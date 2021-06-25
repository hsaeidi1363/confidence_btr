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


float vec_len(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_);

float norm(geometry_msgs::Vector3 & a_);

// calculate the distance of point x0 to the line formed by points x1 and x2
float dist_from_line(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1,geometry_msgs::Vector3 x2);

// check if the point x0 is between the cylinder caps formed around the line from x1 to x2 (cylinder height precalculated for reduced computations)
bool between_caps(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1,geometry_msgs::Vector3 x2, float h);

float calc_density(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd);


float calc_noise(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd);

//fits a plane to the cropped point cloud to later estimate the visibility of path. the plane equation: ax+by+c = z (it does not have 4 params!)
void fit_plane(float & _a, float & _b, float & _d, pcl::PointCloud<pcl::PointXYZI> pcd);

void show_plane(float & _a, float & _b, float & _d, pcl::PointCloud<pcl::PointXYZI> & pcd);

float mid_point_dist(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_);

#endif

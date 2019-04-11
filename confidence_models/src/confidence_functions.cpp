#include"confidence_functions.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami
#include<geometry_msgs/Vector3.h>
#include<Eigen/Dense>
#include <pcl/filters/statistical_outlier_removal.h>



using namespace std;

// cross product of two 3D vectors
geometry_msgs::Vector3 cross_product(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;

  c.x = a_.y*b_.z - a_.z*b_.y;
  c.y = a_.z*b_.x - a_.x*b_.z;
  c.z = a_.x*b_.y - a_.y*b_.x;

  return c;
}

// subtraction of two 3D vectors A-B
geometry_msgs::Vector3 subt(geometry_msgs::Vector3 & a_ , geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;
  c.x = a_.x - b_.x;
  c.y = a_.y - b_.y;
  c.z = a_.z - b_.z;

  return c;
}

// finding the vector length between points A and B
float vec_len(geometry_msgs::Vector3 & a_ , geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;
  c.x = a_.x - b_.x;
  c.y = a_.y - b_.y;
  c.z = a_.z - b_.z;

  return norm(c);
}

float norm(geometry_msgs::Vector3 & a_){

  return sqrt(a_.x*a_.x + a_.y*a_.y + a_.z*a_.z);

}

// distance of a point x0 from the line formed by x1 and x2
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

// finds the number of points in the point cloud that fit in a cylinder between a two marker positions (i.e. start and end points)
float calc_density(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd){
	pcl::PointCloud<pcl::PointXYZI>::iterator pi=pcd.begin();
	float density = 0.0;
	int ctr = 0;
  	float r_c = 0.005;//radius of the cylinder for finding point dentisty between two points
	for( ; pi!=pcd.end(); pi++ ) {

		geometry_msgs::Vector3 x0;
		x0.x = pi->x;
		x0.y = pi->y;
		x0.z = pi->z;
	
		float distance = 0.0;
		distance = dist_from_line(x0, x1, x2);
  
		if (distance <= r_c)
			density += 1.0;
 		ctr ++;
	}

 // double h = vec_len(x1, x2);
 // double cyl_volume = M_PI*r_c*r_c*h;//volume of the cylinder formed by x1-x2 line and radius r_c
 // cout << "volume was: " << cyl_volume<< " for h= "<< h << endl; 
	return density/pcd.points.size();
}

// estimate the noise level on the points inside a cylinder between points x1 and x2
float calc_noise(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZI> pcd){
	pcl::PointCloud<pcl::PointXYZI>::iterator pi=pcd.begin();
	pcl::PointCloud<pcl::PointXYZ> cloud_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
	
	float r_c = 0.005;//radius of the cylinder for finding point dentisty between two points
	for( ; pi!=pcd.end(); pi++ ) {

		geometry_msgs::Vector3 x0;
		x0.x = pi->x;
		x0.y = pi->y;
		x0.z = pi->z;
	
		float distance = 0.0;
		distance = dist_from_line(x0, x1, x2);
  
		if (distance <= r_c){
			pcl::PointXYZ tmp;
			tmp.x = x0.x;
			tmp.y = x0.y;
			tmp.z = x0.z;
			cloud_in.points.push_back(tmp);
	 	}
	}
	cloud_out = cloud_in.makeShared();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true); // Initializing with true will allow us to extract the removed indices
	sorfilter.setInputCloud(cloud_out);
	sorfilter.setMeanK(20);
	sorfilter.setStddevMulThresh(0.5);
	sorfilter.filter (*cloud_out);
	// The resulting cloud_out contains all points of cloud_in that have an average distance to their 8 nearest neighbors that is below the computed threshold
	// Using a standard deviation multiplier of 1.0 and assuming the average distances are normally distributed there is a 84.1% chance that a point will be an inlier
	 pcl::IndicesConstPtr indices_rem = sorfilter.getRemovedIndices();
	// The indices_rem array indexes all points of cloud_in that are outliers

	return (float)indices_rem->size()/cloud_in.points.size();
}

//reference for plane fitting math: https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
void fit_plane(float & _a, float & _b, float & _d,  pcl::PointCloud<pcl::PointXYZI> pcd){
  int pts_len = pcd.points.size();

  Eigen::MatrixXd A(pts_len, 3);
  Eigen::VectorXd B(pts_len); 
  Eigen::VectorXd C(3);
  Eigen::MatrixXd A_p(3,3);

	pcl::PointCloud<pcl::PointXYZI>::iterator pi=pcd.begin();

  int ctr = 0;

	for( ; pi!=pcd.end(); pi++ ) {
    A(ctr, 0) = pi->x; 
    A(ctr, 1) = pi->y; 
    A(ctr, 2) = 1;
    B(ctr) = pi->z;
 
    ctr ++;
  }

  A_p = A.transpose()*A;
  A_p = A_p.inverse()*A.transpose();
  C = A_p*B;
  _a = C(0);
  _b = C(1);
  _d = C(2);

}

void show_plane(float & _a, float & _b, float & _d,  pcl::PointCloud<pcl::PointXYZI> & pcd){

  pcd.points.clear();

  for (float x = -0.05; x <= 0.05; x+= 0.002){
    for(float y = -0.05; y <= 0.05; y+= 0.002){
      pcl::PointXYZI pt;
      pt.x = x;
      pt.y = y;
      pt.z = _a*x + _b*y +_d;
      pt.intensity = 10;
      pcd.points.push_back(pt);
    }
  }

}

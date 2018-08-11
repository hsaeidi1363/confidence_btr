// "Copyright [2017] <Michael Kam>"
/** @file pclMlsSmoothing.cpp
 *  @brief This is the implementation of the pclMlsSmoothing class. This class consists of 5 methods.
 *  Please refer the pclMlsSmoothing.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclMlsSmoothing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclMlsSmoothing is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclMlsSmoothing.  If not, see <http://www.gnu.org/licenses/>.
 */
#include<ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pclMlsSmoothing.h>
#include <ios>
#include <iostream>
// using namespace pcl;

pclMlsSmoothing::pclMlsSmoothing() {
  searchRadius = 0;
}

void pclMlsSmoothing::setSearchRadius(double radius) {
  searchRadius = radius;
}
double pclMlsSmoothing::getSearchRadius() {
  return searchRadius;
}
void pclMlsSmoothing::setInputCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloudIn) {
  cloud = cloudIn;
}
void pclMlsSmoothing::getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudOut = cloud;
}
void pclMlsSmoothing::mlsProcess(
    pcl::PointCloud<pcl::PointNormal>& mls_points) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);
  cloudPtr = cloud.makeShared();
  // Set parameters
  mls.setInputCloud(cloudPtr);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(searchRadius);
  // Reconstruct
  ROS_INFO(" 5d. Smoothing the point cloud, please wait...");
  mls.process(mls_points);
  ROS_INFO(" 5e. Smoothing the point cloud, please wait...");
}

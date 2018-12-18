// "Copyright [2017] <Michael Kam> and <Hamed Saeidi>"
/** @file pcl_waypoints.cpp
 *  @brief This pcl_waypoints.cpp is a ros node that subscribes point cloud and
 *  computes the trajectory f
 *
 *  @author Michael Kam (michael081906) and Hamed Saeidi (hsaeidi1363)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pcl_waypoints is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  talker is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with talker.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <s_hull_pro.h>
#include <pclIo.h>
#include <pclVoxel.h>
#include <pclMlsSmoothing.h>
#include <pclPassThrough.h>
#include <pclStatisticalOutlierRemoval.h>
#include <pclFastTriangular.h>
#include <pclCloudViewer.h>
#include <findNearestPoint.h>
#include <delaunay3.h>
#include <dijkstraPQ.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tissue_waypoints/Trajectory.h>  // This header name name from the project name in CmakeList.txt, not physical folder name
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#include <time.h>
#include <set>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <hash_set>
#include <iostream>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include <std_msgs/Float32.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud pointcloud_in;

/** @brief callback is a callback function that subscribes the point cloud data
 * and uses tf to transform to world coordinate
 *  @param[in] cloud sensor_msgs::PointCloud2Ptr that contains point cloud information
 *  @return none
 */
void callback(const sensor_msgs::PointCloud2Ptr& cloud) {

  pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud

}
/** @brief get_joints is a callback function that subscribe the joint position data
 * and set it into joints vector
 *  @param[in] data sensor_msgs::JointState that contains joint position
 *  @return none
 */

/** @brief find is a service for computing trajectory based on a start and an end point.
 *  @param[in] req is a request that contains coordinates of start and end point.
 *  @param[in] res is a response that contains trajectory information
 *  @return none
 */

ros::Publisher dbg_pub;
float x_min =0.0;
float x_max =0.0;
float y_min =0.0;
float y_max =0.0;
float z_min =0.0;
float z_max =0.0;




bool find(tissue_waypoints::Trajectory::Request &req,
          tissue_waypoints::Trajectory::Response &res) {

  // 0. Initialization

  pclIo pclLoad;
  pclCloudViewer pclView;
  pclPassThrough pt;
  pclVoxel vx;
  pclStatistOutRev sor;
  pclMlsSmoothing mls;
  pclFastTriangular ft;
  pcl::PolygonMesh triangles;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
  cloud_out = pointcloud_in.makeShared();
  pt.setInputCloud(*cloud_out);

  pt.setFilterXlimit(x_min , x_max );

  pt.filterProcess(*cloud_out);
  pt.setInputCloud(*cloud_out);
  pt.setFilterZlimit(z_min, z_max);
  pt.filterProcess(*cloud_out);
  pt.setInputCloud(*cloud_out);
  pt.setFilterYlimit(y_min , y_max);
  pt.filterProcess(*cloud_out);
  pcl::PointCloud<pcl::PointXYZ> dbg_pcd;
  dbg_pcd = *cloud_out;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = std::string("camera_color_optical_frame");
  dbg_pcd.header = pcl_conversions::toPCL(header);
  dbg_pub.publish(dbg_pcd);

  float x_start = req.start[0];
  float y_start = req.start[1];
  float z_start = req.start[2];
  float x_end = req.end[0];
  float y_end = req.end[1];
  float z_end = req.end[2];
  //ROS_INFO(" GOT THIS START AND END POINTS");
  //ROS_INFO("%f ---- %f----%f",x_start,y_start,z_start);
  //ROS_INFO("%f ---- %f----%f",x_end,y_end,z_end);

  float short_x_min = std::min(x_start,x_end);
  float short_y_min = std::min(y_start,y_end);
  float short_z_min = std::min(z_start,z_end);
  float short_x_max = std::max(x_start,x_end);
  float short_y_max = std::max(y_start,y_end);
  float short_z_max = std::max(z_start,z_end);
  //ROS_INFO(" GOT THIS MAX AND MIN VALUES");
  //ROS_INFO("%f ---- %f----%f",short_x_min,short_y_min,short_z_min);
  //ROS_INFO("%f ---- %f----%f",short_x_max,short_y_max,short_z_max);
  float short_offset = 0.003;
  short_x_min -= short_offset;
  short_y_min -= short_offset;
  short_z_min -= short_offset;
  short_x_max += short_offset;
  short_y_max += short_offset;
  short_z_max += short_offset;
  ///ROS_INFO(" ADDED OFFSET TO MAX AND MIN VALUES");
  //ROS_INFO("%f ---- %f----%f",short_x_min,short_y_min,short_z_min);
  //ROS_INFO("%f ---- %f----%f",short_x_max,short_y_max,short_z_max);
  pt.setFilterXlimit(short_x_min, short_x_max);
  //ROS_INFO(" CROPPED FOR THE SECOND TIME");
  pt.filterProcess(*cloud_out);
  pt.setInputCloud(*cloud_out);
  pt.setFilterZlimit(z_min, z_max);
  pt.filterProcess(*cloud_out);
  pt.setInputCloud(*cloud_out);
  pt.setFilterYlimit(short_y_min , short_y_max );
  pt.filterProcess(*cloud_out);

  


  //  4. Down sample the point cloud
  //ROS_INFO(" 4. Down sampling the point cloud, please wait...");
  //vx.setInputCloud(*cloud_out);
  //vx.setLeafSize(0.0015, 0.0015, 0.0015);
  //vx.filterProcess(*cloud_out);
  
  //ROS_INFO(" Down sampling completed");
  /****************************************************************************
   pcl::visualization::CloudViewer viewer("Cloud of Raw data");
   viewer.showCloud(cloud_out);
   int user_data = 0;
   do {
   user_data++;
   } while (!viewer.wasStopped());
   // ****************************************************************************/
  // pub_pcl.publish(cloud_out);
  //  2. Remove the noise of point cloud
//	TODO: do we need noise reduction?!
 /* ROS_INFO("2. Removing the noise of point cloud, please wait...");
  sor.setInputCloud(*cloud_out);
  sor.setMeanK(10);
  sor.setStddevMulThresh(1);
  sor.filterProcess(*cloud_out);
*/

  //*******************************************************************
  // 2017.11.9 Added Michael
  /*pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud_out);
  outrem.setRadiusSearch(0.5);
  outrem.setMinNeighborsInRadius(30);
  // apply filter
  outrem.filter(*cloud_out);
  */
  //*******************************************************************
  ROS_INFO(" Removing completed");
  /*****************************************************************/

  //  3. Extract certain region of point cloud
  //  5. Smooth the point cloud
  // pcl::io::savePCDFile("resultsstep5.pcd",*cloud_out, true);

  ROS_INFO(" 5. Smoothing the point cloud, please wait...");

  //mls.setInputCloud(*cloud_out);
  //ROS_INFO(" 5a. Smoothing the point cloud, please wait...");
  //mls.setSearchRadius(0.01);
  //ROS_INFO(" 5b. Smoothing the point cloud, please wait...");
  //mls.mlsProcess(*cloud_with_normal);
  //ROS_INFO(" 5c. Smoothing the point cloud, please wait...");

  //  7. Mesh the obstacle
  /*ROS_INFO(" 7. Meshing the obstacle, please wait...");
  ft.setInputCloud(*cloud_with_normal);
  ft.setSearchRadius(0.05);
  ft.reconctruct(triangles);
  std::vector<int> gp33 = ft.getSegID();
  int sizegp3 = gp33.size();
  int sizePointCloud = cloud_with_normal->size();
*/
  //  8. Show the result
  // pclView.display(triangles);

  // 9.mergeHanhsPoint
  ROS_INFO(" 9. Merging point of selected point, please wait...");
  findNearestPoint mg;
  std::vector<int> selectedPoint;

  // mg.readtext("./src/Robotic-polishing/kidney3dots3_pointCluster.txt");
  mg.setPosition(req.start);
  mg.setPosition(req.end);

  mg.setInputCloud(*cloud_out);
  mg.findNearestProcess(selectedPoint);
  std::vector<int> size3;
 // size3 = ft.getSegID();
  ROS_INFO(" 10. Delaunay3 function...");
  std::vector<Triad> triads;
  // std::vector<Shx> ptsOut;
  delaunay3 dy3;
  dy3.setInputCloud(*cloud_out);


  ROS_INFO(" 10.b.1");
  dy3.putPointCloudIntoShx();
  ROS_INFO(" 10.b.2");
  dy3.processDelaunay(triads);
  ROS_INFO(" 10.b.3");
 
  // dy3.getShx(ptsOut);
  // write_Triads(triads, "triangles.txt");
  // write_Shx(ptsOut, "pts.txt");
  //std::cout<<selectedPoint[0]<<"  "<< selectedPoint[1]<<std::endl;
  std::cout<<triads.size()<<std::endl;
  int start = selectedPoint[0];
  int end = selectedPoint[1];
  std::vector<int> path;
  dijkstraPQ dPQ(triads.size());
  dPQ.setInputCloud(*cloud_out);
  dPQ.setTri(triads);
  dPQ.computeWeight();
  dPQ.shortestPath(start, end);
  dPQ.returnDijkstraPath(start, end, path);

 
  //  std::cout << std::endl;
  std::vector<position> POS;
  dPQ.returnDijkstraPathPosition(start, end, POS);
  std::vector<position>::iterator routePos = POS.begin();

  float px, py, pz;
  routePos = POS.begin();
  while (routePos != POS.end()) {
    px = (*routePos).x;
    res.path_x.push_back(px);
    py = (*routePos).y;
    res.path_y.push_back(py);
    pz = (*routePos).z;
    res.path_z.push_back(pz);

    ++routePos;
  }

//---------------------------------------------------//
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "tissue_waypoints");
  ros::NodeHandle n;
  ros::NodeHandle home("~");
  home.getParam("x_min", x_min);
  home.getParam("x_max", x_max);
  home.getParam("y_min", y_min);
  home.getParam("y_max", y_max);
  home.getParam("z_min", z_min);
  home.getParam("z_max", z_max);

 

  // ros::Publisher pub_pcl = nh.advertise<PointCloud>("test", 10);
  ros::Subscriber sub_pcl = n.subscribe("/d415/filtered_points", 1, &callback);
  dbg_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>>("/d415/passthrough",1);
  ros::ServiceServer service = n.advertiseService("FindTrajectory", &find);

  ros::Rate loop_rate(10);


  while (ros::ok()) {
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

// "Copyright [2017] <Michael Kam>"
/** @file findNearestPoint.cpp
 *  @brief This findNearestPoint.cpp is an implementation of finding
 *   the closest point on a post smoothing surface. There are three method in this class.
 *   The goal is to get indices array which indicate the closest point with several given points.
 *   For example, I can given a arbitrary coordinate and use this class to
 *   find a closest point ID from point cloud group.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  findNearestPoint is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  findNearestPoint is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with findNearestPoint.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <findNearestPoint.h>
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <ctime>

findNearestPoint::findNearestPoint() {
}

std::vector<double> findNearestPoint::readtext(const std::string &fileName) {
  // TODO(Michael): remember how to load file in C++
  std::ifstream file(fileName);  // Create a input stream
  std::vector<std::string> readString;
  std::vector<double> readDouble;
  std::string word;
  while (file >> word) {
    readString.push_back(word);  // push back string, which defined in the vector
    // TODO(Michael): remember how to convert string into double
    this->specificPoints.push_back(atof(word.c_str()));
    readDouble.push_back(atof(word.c_str()));
  }
  return readDouble;
}

void findNearestPoint::setPosition(const std::vector<float>& pos) {
  for (std::vector<float>::const_iterator it = pos.begin(); it < pos.end();
      ++it) {
    this->specificPoints.push_back(*it);
  }
}

void findNearestPoint::setInputCloud(
    pcl::PointCloud<pcl::PointXYZ>& cloudIn, bool *tri_) {
  cloud = cloudIn;
  tri_point = tri_;
}

void findNearestPoint::findNearestProcess(std::vector<int>& nearIndices) {
  // Set pointNormal as an input into kTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());
  // Create the point cloud you want to search
  pcl::PointXYZ searchPoint;
  pcl::PointCloud<pcl::PointXYZ> searchPoints;
  int i;
  // TODO(Michael): There must be a better way to do this
  /*  for (std::vector<double>::const_iterator iter = specificPoints.begin();
   iter != specificPoints.end(); iter + 3) {
   searchPoint.x = *iter;
   searchPoint.y = *(iter + 1);
   searchPoint.z = *(iter + 2);
   searchPoints.push_back(searchPoint);
   } */
  // 2017.11.26 [Bug fixed] Michael. ++++++iter
  for (std::vector<double>::const_iterator iter = specificPoints.begin();
      iter < specificPoints.end(); ++++++iter) {
    searchPoint.x = *iter;
    searchPoint.y = *(iter + 1);
    searchPoint.z = *(iter + 2);
    searchPoints.push_back(searchPoint);
  }
  // Number of K point you want to search
  int K = 20;
  // Vector to store point index
  std::vector<int> pointIdxNKNSearch(K);
  // Vector to store distance between given point and Kth near point
  std::vector<float> pointNKNSquaredDistance(K);
  // const_iterator: if you only want to read
  // iterator: if you want to write
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = searchPoints
      .begin(); iter != searchPoints.end(); iter++) {
    kdtree.nearestKSearch(*iter, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    // 2019/4/26 Michael
    for(int nn=0;nn<pointIdxNKNSearch.size();nn++)
    {
      if(tri_point[pointIdxNKNSearch[nn]])
      {
        nearIndices.push_back(pointIdxNKNSearch[nn]);
      }
    }
    //nearIndices.insert(nearIndices.end(), pointIdxNKNSearch.begin(),pointIdxNKNSearch.end());
  }
  if(nearIndices.size()==0)
  std::cout << "nearIndices.size()= " << nearIndices.size() << std::endl;

}

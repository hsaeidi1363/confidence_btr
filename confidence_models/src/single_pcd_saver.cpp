#include<ros/ros.h>
#include <ros/package.h>
#include<fstream>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include<pcl_ros/point_cloud.h>




bool pcl_available = false;

pcl::PointCloud<pcl::PointXYZRGB> pointcloud_in;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud) {

  pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  pcl_available = true;
}

int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "single_pcd_saver");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	
	std::string test_no = "dbg";
	home.getParam("test_no", test_no);





	ros::Subscriber sub_pcl = nh.subscribe("/d415/filtered_points", 1, get_pcl);


	while(!pcl_available){	
		  ros::spinOnce();
		  ros::Duration(0.5).sleep(); 
		  ROS_INFO("waiting for pcl"); 	  
	}
//	pcl::io::savePCDFileASCII ((ros::package::getPath("confidence_models") + "/ident_data/"+test_no+".pcd").c_str(), pointcloud_in);
	pcl::io::savePCDFileASCII (("/home/hsaeidi/ident_data/"+test_no+".pcd").c_str(), pointcloud_in);
	std::cerr << "Saved " << pointcloud_in.size () << " data points " << std::endl;

	
	ROS_INFO("!!!!!!!Finished logging pcl!!!!!!!");	

 
	ROS_INFO("!!!!!!!Finished logging all data =>>> shtting down!!!!!!!");
	return 0;

}

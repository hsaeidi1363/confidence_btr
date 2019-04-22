#include<ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include<fstream>
#include <geometry_msgs/Polygon.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include<pcl_ros/point_cloud.h>


geometry_msgs::Polygon markers3D;

bool poly_available = false;
void get_polygon(const geometry_msgs::Polygon & _data){
	markers3D = _data;	
	poly_available = true;
	int len = markers3D.points.size();
	ROS_INFO("got a new reading from 3D polygons with %d points",len);
}

bool pcl_available = false;

pcl::PointCloud<pcl::PointXYZ> pointcloud_in;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud) {

  pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  pcl_available = true;
}

int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "identification_data");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	
	std::string test_no = "dbg";
	home.getParam("test_no", test_no);



    tf::TransformListener listener;
	ros::Subscriber polygon_sub = nh.subscribe("/nir_overlay_intel/polygon3D_cog",1, get_polygon);
	ros::Subscriber sub_pcl = nh.subscribe("/d415/filtered_points", 1, get_pcl);

	bool basler_intel_read =false;
	while(!basler_intel_read){
		try {  // get the pos/ori of raytrix wrt basler
			  
			  std::ofstream positions_file;
			 // positions_file.open((ros::package::getPath("confidence_models") + "/ident_data/"+test_no+"_frames.csv").c_str());
			  positions_file.open(("/home/hsaeidi/ident_data/"+test_no+"_frames.csv").c_str());
			  positions_file << "frame1_in_frame2,x,y,z,r,p,y" << std::endl;
			  
			  tf::StampedTransform tfRt1;
			  tf::StampedTransform tfRt2;
			  tf::StampedTransform tfRt3;
			  tf::StampedTransform tfRt4;
			  tf::StampedTransform tfRt5;
			  tf::StampedTransform tfRt6;
			  listener.lookupTransform( "basler", "intel", ros::Time(0), tfRt1 );
			  listener.lookupTransform( "intel", "basler", ros::Time(0), tfRt2 );
			  listener.lookupTransform( "kuka", "intel", ros::Time(0), tfRt3 );
			  listener.lookupTransform( "kuka", "basler", ros::Time(0), tfRt4 );
			  listener.lookupTransform( "kuka", "identification_pattern", ros::Time(0), tfRt5 );
			  listener.lookupTransform( "intel", "identification_pattern", ros::Time(0), tfRt6 );



			  double x = 0.0; 
			  double y = 0.0;
			  double z = 0.0; 
			  double roll = 0.0;
			  double pitch = 0.0;
			  double yaw = 0.0;
			  //tf::Quaternion q; fix later
			  x = tfRt1.getOrigin()[0];
			  y = tfRt1.getOrigin()[1];
			  z = tfRt1.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt1.getRotation()).getRPY(roll, pitch, yaw);
			  
			  positions_file <<"intel_in_balser"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;

			  x = tfRt2.getOrigin()[0];
			  y = tfRt2.getOrigin()[1];
			  z = tfRt2.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt2.getRotation()).getRPY(roll, pitch, yaw);
			  positions_file <<"basler_in_intel"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;

			  x = tfRt3.getOrigin()[0];
			  y = tfRt3.getOrigin()[1];
			  z = tfRt3.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt3.getRotation()).getRPY(roll, pitch, yaw);
			  positions_file <<"intel_in_kuka"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;

			  x = tfRt4.getOrigin()[0];
			  y = tfRt4.getOrigin()[1];
			  z = tfRt4.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt4.getRotation()).getRPY(roll, pitch, yaw);
			  positions_file <<"basler_in_kuka"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;

			  x = tfRt5.getOrigin()[0];
			  y = tfRt5.getOrigin()[1];
			  z = tfRt5.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt5.getRotation()).getRPY(roll, pitch, yaw);
			  positions_file <<"pattern_in_kuka"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;

			  x = tfRt6.getOrigin()[0];
			  y = tfRt6.getOrigin()[1];
			  z = tfRt6.getOrigin()[2];		  
			  tf::Matrix3x3(tfRt6.getRotation()).getRPY(roll, pitch, yaw);
			  positions_file <<"pattern_in_intel"<<","<< x<< ","<< y << ","<< z<< ","<< roll << "," <<pitch<< ","<< yaw<<std::endl;


			  
			  
			  positions_file.close();
			 

			  basler_intel_read = true;
			}
			catch(...) { 
			  ros::Duration(0.5).sleep(); 
			  ROS_INFO("waiting for basler_intel frame"); 
			  
			}

	}
	ROS_INFO("!!!!!!!Finished logging frames!!!!!!!");


	while(!poly_available){	
		  ros::spinOnce();
		  ros::Duration(0.5).sleep(); 
		  ROS_INFO("waiting for polygons"); 
		  
	}
	std::ofstream markers3D_file;
//	markers3D_file.open((ros::package::getPath("confidence_models") + "/ident_data/"+test_no+"_markers3D.csv").c_str());
	markers3D_file.open(("/home/hsaeidi/ident_data/"+test_no+"_markers3D.csv").c_str());
	markers3D_file << "x,y,z" << std::endl;
	for (int i = 0; i < markers3D.points.size(); i++){
		markers3D_file<< markers3D.points[i].x<< ","<< markers3D.points[i].y << ","<< markers3D.points[i].z<<std::endl;
	}
	markers3D_file.close();
	ROS_INFO("!!!!!!!Finished logging markers!!!!!!!");	
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

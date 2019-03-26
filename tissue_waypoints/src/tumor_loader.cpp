#include <ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<iostream>
#include <std_msgs/Bool.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami


#include <sensor_msgs/PointCloud2.h>

using namespace std;


void check_files(ifstream& in_file,string& in_name){
	if(!in_file.is_open()){
		cerr<< "Cannot open trajectory file"<< in_name<< endl;
		exit(EXIT_FAILURE);
	}	
}

bool send_plan = false;

void update_plan(const std_msgs::Bool & _data){
	send_plan = _data.data;
	std::cout << "got a command for sending plan \n"; 
}

int main( int argc, char** argv ){

    
  string plan_file_name;
  ros::init( argc, argv, "nir_emulator" );

  ros::NodeHandle nh, nhp("~");
  ros::Rate loop_rate(10);
  ros::Subscriber sub_send_plan = nh.subscribe("/send_plan", 1, update_plan);
  ros::Publisher    pub_cog=nh.advertise< pcl::PointCloud<pcl::PointXYZI> >("/plancloud", 1);
  pcl::PointCloud<pcl::PointXYZI> markers;
  pcl::PointXYZI xyzi;
  
  
  nhp.getParam("plan_file_name", plan_file_name);
  plan_file_name = ros::package::getPath("star_rtt") + plan_file_name;
  ifstream plan_file(plan_file_name.c_str(), ifstream::in);
  check_files(plan_file,plan_file_name);
  string line;
  int ctr = 1;
  while(getline(plan_file, line)){
    istringstream iss(line);
    // read the positions
    double scale = 0.0;
    iss >> xyzi.x;
    iss >> xyzi.y;
    iss >> xyzi.z;
    if (fabs(xyzi.y) < 1)
        scale = 1.0; // if the readings are in meters
    else
        scale = 0.001; // scale the readings from millimiter to meters
    xyzi.x *= scale;
    xyzi.y *= scale;
    xyzi.z *= scale;
    xyzi.intensity = 1.0;
    markers.push_back( xyzi );
    cout << "read coordinates for point: " << ctr<<" at: x= "<< xyzi.x << "(m) y= "<<xyzi.y << "(m) and z = "<< xyzi.z << "(m)"<< endl;
    ctr ++;
  }
	
  /*
  xyzi.x = -0.1129;
  xyzi.y = 0.5236;
  xyzi.z = -0.3295;
  
  xyzi.x = -0.1129;
  xyzi.y = 0.4936;
  xyzi.z = -0.3295;
  */
  
  
  int seq = 0;
  while(ros::ok()){
	if(send_plan){
	      std_msgs::Header header;
	      header.stamp = ros::Time::now();
	      header.seq = seq++;
	      header.frame_id = std::string( "/camera_color_optical_frame" );
		
	      markers.header = pcl_conversions::toPCL( header );
	      pub_cog.publish(markers);
	      send_plan = false;
	      std::cout << "sent a plan \n";
	}
	loop_rate.sleep();
	ros::spinOnce();
  }
  plan_file.close();
  return 0;

}
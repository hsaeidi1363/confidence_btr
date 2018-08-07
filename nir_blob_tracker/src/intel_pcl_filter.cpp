/****************************************************************************
 * TODO: change the following
 * nirtracker.cpp 
 * 04-05-2013 
 *
 * Description:
 *
 * Implements tracking node based on ViSP vpDot "blob" trackers. 
 * 
 * Tested on NIR images obtained from JAI and Basler cameras, processed by nirthreshold.cpp
 * 
 * 08-12-2013 rev:
 * Fixes publishing of cog, moments. Adds publishing of edges (not tested)
 *
 * Authors:
 * Azad Shademan
 * a.shademan@ieee.org
 *
 *****************************************************************************/


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>



class IntelPCLFilter{


public:

  bool 							  laser_off;
  double 						  t0;
  double 						  t1;
  ros::Subscriber sub_intel_laser;
  ros::Subscriber sub_pcl;
  ros::Publisher pub_pcl;
  
  IntelPCLFilter( ros::NodeHandle& nh ){
    
    sub_intel_laser = nh.subscribe("d415_laser_off", 1 , &IntelPCLFilter::CallbackLaser, this);
	
    sub_pcl = nh.subscribe("camera/depth_registered/points", 10, &IntelPCLFilter::pcl_callback, this);

    pub_pcl= nh.advertise<sensor_msgs::PointCloud2>( "d415/filtered_points", 10 );

  } 
  
  ~IntelPCLFilter(){}


   void CallbackLaser(const std_msgs::BoolConstPtr& msg){
    laser_off = msg->data;
    t0 = ros::Time::now().toSec();
   }
   void pcl_callback(const sensor_msgs::PointCloud2ConstPtr cloud) {
  // copy sensor_msg::Pointcloud2Ptr message into pcl::PointCloud pointcloud_out
		t1 = ros::Time::now().toSec() -t0;		
		if(!laser_off && (t1 > 0.22)){
			pub_pcl.publish(cloud);
		}

	}

};

/*  
 * =============================
 */
int main( int argc, char *argv[]){
  
  ros::init( argc, argv, "intel_pcl_filter");
  ros::NodeHandle nh;

  IntelPCLFilter filter( nh );
  ros::spin();
  
  return 0;
}

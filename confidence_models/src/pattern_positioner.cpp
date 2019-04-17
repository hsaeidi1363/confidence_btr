#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "pattern_positioner");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	ros::Rate loop_rate(1);
	
//add some position and angle readings here


	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	ros::Publisher coordinate_pub = nh.advertise<geometry_msgs::Twist>("/pattern_coordinates",1);


	bool basler_intel_read =false;
	while(ros::ok()){
		try {  // get the pos/ori of raytrix wrt basler
			  
			 
			  
			  tf::StampedTransform tfRt1;
			  listener.lookupTransform( "kuka", "intel", ros::Time(0), tfRt1 );

			  double x = 0.0; 
			  double y = 0.0;
			  double z = 0.0; 
			  //double roll = 0.0;
			  //double pitch = 0.0;
			  //double yaw = 0.0;
			  //tf::Quaternion q; fix later
			  x = tfRt1.getOrigin()[0];
			  y = tfRt1.getOrigin()[1];
			  z = tfRt1.getOrigin()[2];		  
			//  tf::Matrix3x3(tfRt1.getRotation()).getRPY(roll, pitch, yaw);

		 	  tf::Transform in_kuka;
		 	  tf::Transform in_cam;
			  tf::Vector3 pos_in_world;
			  pos_in_world = tfRt1(tf::Vector3(0.0,0.0,0.25));
			  in_kuka.setOrigin( pos_in_world );
			  tf::Quaternion q_in_kuka;
			  tf::Quaternion q_in_cam;
			  q_in_cam.setRPY(0.0, 3.1415, 0.79);
			  q_in_kuka = tfRt1*(q_in_cam);
			  in_kuka.setRotation(q_in_kuka);
			  br.sendTransform(tf::StampedTransform(in_kuka, ros::Time::now(), "kuka", "pattern_motion"));		

		}
		catch(...) { 
		  ROS_INFO("waiting for frames"); 
		  
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

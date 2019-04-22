#include<ros/ros.h>
#include<ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include<fstream>

double deg2rad(double x) { return x * 3.1415 / 180;}

int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "pattern_positioner");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	ros::Rate loop_rate(1);
	double x = 0.0; 
	double y = 0.0;
	double z = 0.0; 
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	home.getParam("roll", roll);
	home.getParam("pitch",pitch);
	home.getParam("yaw",yaw);
	home.getParam("x",x);
	home.getParam("y",y);
	home.getParam("z",z);

	roll = deg2rad(roll);
	pitch = deg2rad(pitch);
	yaw = deg2rad(yaw);
	
//add some position and angle readings here


	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	ros::Publisher coordinate_pub = nh.advertise<geometry_msgs::Twist>("/pattern_coordinates",1);

	std::ofstream positions_file;
	positions_file.open((ros::package::getPath("confidence_models") + "/pattern_coordinates/coordinates.txt").c_str());

	bool file_saved =false;
	while(ros::ok()){
		try {  // get the pos/ori of raytrix wrt basler
			  
			 
			  
			  tf::StampedTransform intel_in_kuka;
			  listener.lookupTransform( "kuka", "intel", ros::Time(0), intel_in_kuka );

		
			  
		 	  tf::Transform pattern_in_kuka;
			  tf::Transform pattern_in_intel;
			  tf::Transform ref_in_intel;
			  tf::Transform intel_in_pattern;

  			  tf::Vector3 pos_pattern_in_intel;
			  pos_pattern_in_intel = tf::Vector3(x,y,z);

			  // convert the relative position of pattern in the intel camera to the kuka frame
			  tf::Vector3 pos_pattern_in_kuka;			 
			  pos_pattern_in_kuka = intel_in_kuka(pos_pattern_in_intel);
			  
			  pattern_in_kuka.setOrigin( pos_pattern_in_kuka );

			  tf::Quaternion q_pattern_in_kuka;
			  tf::Quaternion q_pattern_in_intel;


			  q_pattern_in_intel.setRPY(0.0, 3.1415,  0.79); // first a roll, then a pitch, and finally a yaw all in the fixed intel frame!

			  tf::Quaternion q_ref_in_intel;
			  q_ref_in_intel.setRPY(roll,pitch, yaw);

 			  tf::Transform tmp;
			  tmp.setRotation(q_ref_in_intel);

			  q_pattern_in_intel = tmp*(q_pattern_in_intel);
 			  q_pattern_in_kuka = intel_in_kuka*(q_pattern_in_intel);

			  pattern_in_kuka.setRotation(q_pattern_in_kuka);
				
			

			
			 
			 
			  


			  br.sendTransform(tf::StampedTransform(pattern_in_kuka, ros::Time::now(), "kuka", "pattern_motion"));	
			  if(!file_saved){
	 			double x_f = pattern_in_kuka.getOrigin()[0];
			 	double y_f = pattern_in_kuka.getOrigin()[1];
			  	double z_f = pattern_in_kuka.getOrigin()[2];	
				double roll_f = 0.0;
				double pitch_f = 0.0;
				double yaw_f = 0.0;	  
				tf::Matrix3x3(pattern_in_kuka.getRotation()).getRPY(roll_f, pitch_f, yaw_f);
				positions_file << ("<param name=\"roll\" value=\""+std::to_string(roll_f)+"\"/>").c_str() << std::endl;
				positions_file << ("<param name=\"pitch\" value=\""+std::to_string(pitch_f)+"\"/>").c_str() << std::endl;
				positions_file << ("<param name=\"yaw\" value=\""+std::to_string(yaw_f)+"\"/>").c_str() << std::endl;
				positions_file << ("<param name=\"x\" value=\""+std::to_string(x_f)+"\"/>").c_str() << std::endl;
				positions_file << ("<param name=\"y\" value=\""+std::to_string(y_f)+"\"/>").c_str() << std::endl;
				positions_file << ("<param name=\"z\" value=\""+std::to_string(z_f)+"\"/>").c_str() << std::endl;
				ROS_INFO("--------------SAVED THE NEW COORDINATES-------------"); 
	  		 	positions_file.close();
				file_saved = true;
			  }	

		}
		catch(...) { 
		  ROS_INFO("waiting for frames"); 
		  
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

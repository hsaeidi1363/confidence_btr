// this codes overlays the 2D position of the NIR markers on the RGB image of intel camera (need to run the nir_overlay.launch before this)
#include <ros/ros.h>
#include<ros/package.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Bool.h>

#include<fstream>

double deg2rad(double x) { return x * 3.1415 / 180;}
double rad2deg(double x) { return x * 180 / 3.1415;}

using namespace std;


bool save_coordinates = false;

void get_save_command(const std_msgs::Bool & _data){
	save_coordinates = _data.data;	
}






int main(int argc, char * argv[]){
	ros::init(argc,argv,"find_manual_pattern");
	ros::NodeHandle nh_;

	ros::NodeHandle home("~");


    int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);

	ros::Subscriber freeze_path_sub = nh_.subscribe("/save_frames",1,get_save_command);	

	ros::Publisher pattern_pub = nh_.advertise<geometry_msgs::Twist>("/pattern_loc",1);

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
	
	ROS_INFO("Read x = %f, y = %f, z= %f, roll =%f, pitch = %f, yaw=%f",x,y,z,roll,pitch,yaw); 

	roll = deg2rad(roll);
	pitch = deg2rad(pitch);
	yaw = deg2rad(yaw);

	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	tf::Vector3 pos_in_ref;



	tf::Vector3 pos_pattern_in_flea;
    //pos_pattern_in_flea = tf::Vector3(x,y,z);
  
	tf::Quaternion q_pattern_in_flea;
    //q_pattern_in_flea.setRPY(roll, pitch,  yaw);

    tf::Transform pattern_in_flea;
	
	tf::Transform pattern_in_checkerboard;

	pattern_in_checkerboard.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion q_pattern_in_checkerboard;
	q_pattern_in_checkerboard.setRPY(deg2rad(0),deg2rad(180),deg2rad(-90));
    pattern_in_checkerboard.setRotation(q_pattern_in_checkerboard);

	//pattern_in_flea.setOrigin(pos_pattern_in_flea);
	//pattern_in_flea.setRotation(q_pattern_in_flea);

	geometry_msgs::Twist pattern; 

	std::ofstream pattern_coordinate_file;
	pattern_coordinate_file.open((ros::package::getPath("confidence_models") + "/scripts/manual_pattern_frame.sh").c_str());

	while(ros::ok()){
 		try {
			tf::StampedTransform tfRt;
			listener.lookupTransform( "camera", "checkerboard", ros::Time(0), tfRt );
		  //  pos_in_ref = transform(tf::Vector3(xyz.linear.x,xyz.linear.y,xyz.linear.z));
			pattern_in_flea = tfRt*(pattern_in_checkerboard);
		

			br.sendTransform(tf::StampedTransform(pattern_in_flea, ros::Time::now(), "camera", "pattern"));	

			pattern.linear.x = (pattern_in_flea.getOrigin()[0] - x)*100;
			pattern.linear.y = (pattern_in_flea.getOrigin()[1] - y)*100;
			pattern.linear.z = (pattern_in_flea.getOrigin()[2] - z)*100;	

			tf::Matrix3x3(pattern_in_flea.getRotation()).getRPY(pattern.angular.x, pattern.angular.y, pattern.angular.z);
		
			pattern.angular.x -= roll;
			pattern.angular.y -= pitch;
			pattern.angular.z -= yaw;

			pattern.angular.x = rad2deg(pattern.angular.x);
			pattern.angular.y = rad2deg(pattern.angular.y);
			pattern.angular.z = rad2deg(pattern.angular.z);

			pattern_pub.publish(pattern);

			if(save_coordinates){
					double x_f = tfRt.getOrigin()[0];
				 	double y_f = tfRt.getOrigin()[1];
				  	double z_f = tfRt.getOrigin()[2];	
					double roll_f = 0.0;
					double pitch_f = 0.0;
					double yaw_f = 0.0;	  
					tf::Quaternion q_tmp;
					q_tmp = tfRt.getRotation();
					pattern_coordinate_file << "#!/bin/bash" << endl;
					pattern_coordinate_file << "rosrun tf static_transform_publisher "<<x_f<< ", "<<y_f<< ", " <<z_f<< " " <<tfRt.getRotation()[0]<<", "<<tfRt.getRotation()[1] <<", "<<tfRt.getRotation()[2] <<", "<<tfRt.getRotation()[3] <<" \"flea\" \"pattern\" 10"<< endl;
	 
	//rosrun tf static_transform_publisher 0.345, 0.030, 0.221 0.679, 0.627, -0.260, -0.281 "checkerboard" "flea" 10

				//	positions_file << ("<param name=\"roll\" value=\""+std::to_string(roll_f)+"\"/>").c_str() << std::endl;
					ROS_INFO("--------------SAVED THE NEW COORDINATES-------------"); 
				
					break;
			}
		}
		catch(...) { 
		  std::cout << "----No FRAME AT THE MOMENT" << std::endl; 
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	pattern_coordinate_file.close();
	return 0;
	
}
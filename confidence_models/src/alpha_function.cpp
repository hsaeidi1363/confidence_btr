#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Bool.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<fstream>
#include <string>


std_msgs::Bool test_start;


trajectory_msgs::JointTrajectory auto_cmd;
trajectory_msgs::JointTrajectory manual_cmd;
trajectory_msgs::JointTrajectory final_cmd;
trajectory_msgs::JointTrajectoryPoint pt;
geometry_msgs::Twist alpha;

bool auto_available = false;
//read the autonomous control command
void get_auto(const trajectory_msgs::JointTrajectory & _data){
	auto_cmd = _data;
	auto_available = true;
   //   ROS_INFO("got auto command");

}


bool manual_available = false;
//read the manual control command
void get_manual(const trajectory_msgs::JointTrajectory & _data){
	manual_cmd = _data;
	manual_available = true;
    //  ROS_INFO("got manual command");
}

geometry_msgs::Twist rob_pos;
bool rob_pos_available = false;

void get_rob_pos(const geometry_msgs::Twist & _data){
   //   ROS_INFO("got robot pose");
      rob_pos = _data;
      rob_pos_available = true;
} 

void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj){
	for (int i = 1; i <= _nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "iiwa_joint_";
		joint_name << i;
		_cmd.joint_names.push_back(joint_name.str());
	}
}

// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}

float c_a = 0.6;
float c_m = 0.6;

// apply alpha
void apply_alpha(){
	float x = rob_pos.linear.x;
	float y = rob_pos.linear.y;

	float sigma = 0.03;
	float mu = 0.55;	
	c_a =exp( -(x - mu)*(x - mu)/(2*sigma*sigma) )/sqrt(2*M_PI*sigma*sigma)/15.3;


	float cur_alpha;
	cur_alpha = 1 - 1/(1 + exp( -5*(c_a - c_m) ) );
	alpha.linear.x = cur_alpha*100;
	alpha.linear.y = c_a;
	alpha.linear.z = c_m;
        for (int i = 0; i < 7; ++i)
		pt.positions[i] = (1 - cur_alpha)*auto_cmd.points[0].positions[i] + cur_alpha*manual_cmd.points[0].positions[i];
	
}

int main(int argc, char * argv[]){
    
    	alpha.linear.x = 100.0; //start with manual initially
	ros::init(argc, argv, "alpha_function");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");
        
	int loop_freq = 100;
        float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	std::string test_no = "dbg";
	home.getParam("test_no", test_no);
	
	std::ofstream data_file;
	data_file.open(("/home/hsaeidi/ident_data/"+test_no+"_data.csv").c_str());
	// should I add haptic device inputs as well?! -----or the commands coming to the robot --------
	data_file << "x,y,z,r,p,y,alpha,c_a,c_m,a0,a1,a2,a3,a4,a5,a6,m0,m1,m2,m3,m4,m5,m6" << std::endl;



	initialize_points(pt,7,0.0);
	name_joints(final_cmd, 7);

        pt.time_from_start = ros::Duration(0.01);
        final_cmd.points.push_back(pt);



	std::string command_topic = "iiwa/PositionJointInterface_trajectory_controller/command";
        ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,10);
        ros::Publisher alpha_pub = nh_.advertise<geometry_msgs::Twist>("alpha",10);

        ros::Subscriber auto_sub = nh_.subscribe("/iiwa/auto/command",10, get_auto);
        ros::Subscriber man_sub = nh_.subscribe("/iiwa/manual/command",10, get_manual);
	ros::Subscriber robot_pos_sub = nh_.subscribe("robot/worldpos", 1, &get_rob_pos);

	while (ros::ok()){
		if (auto_available && manual_available && rob_pos_available){
		//if(rob_pos_available){
			apply_alpha();
                        final_cmd.points[0] = pt;
                        final_cmd.header.stamp = ros::Time::now();
			cmd_pub.publish(final_cmd);
			alpha_pub.publish(alpha);
			data_file << rob_pos.linear.x <<","<< rob_pos.linear.y <<","<< rob_pos.linear.z <<","<< rob_pos.angular.x <<","<< rob_pos.angular.y<<","<< rob_pos.angular.z << ","<<alpha.linear.x <<","<<alpha.linear.y<<","<< alpha.linear.z;
			for (int i = 0; i < 7; ++i)
				data_file << ","<< auto_cmd.points[0].positions[i];
			for (int i = 0; i < 7; ++i)
				data_file << ","<< manual_cmd.points[0].positions[i];
			data_file <<std::endl;
		}
			
        	ros::spinOnce();
		loop_rate.sleep();
	}

	data_file.close();
	return 0;
}


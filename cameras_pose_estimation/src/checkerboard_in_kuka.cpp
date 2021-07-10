#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iiwa_msgs/JointPosition.h>
#include<sensor_msgs/JointState.h>
#include<kdl/chain.hpp>
#include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>

//Frame KDL::Frame::DH_Craig1989 (double a, double alpha, double d, double theta)


double needle_length = 0.0;

KDL::Chain LWR(){

  double tool_length = 0.506;
  double total_tool_length = tool_length + needle_length + 0.12597;//0.12597 m from joint to flange 

  KDL::Chain chain;

  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(0,0,0.33989,0)));

  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 2 
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0.40011,0)));

  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));

  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0.40003,0)));

  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, M_PI_2,0,0)));

  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
  KDL::Frame::DH_Craig1989(0,0,total_tool_length,0)));

  return chain;

}

//reading the kuka lwr joint positions
sensor_msgs::JointState joints;
bool initialized = false;
//callback for reading joint values
void get_joints(const sensor_msgs::JointState & data){
	for (int i = 0; i < data.position.size();++i){
		// if this is not the first time the callback function is read, obtain the joint positions
		if(initialized){
			joints.position[i] = data.position[i];	
		// otherwise initilize them with 0.0 values
		}else{
			joints.position.push_back(0.0);
		}
	}	
	initialized = true;
}





int main(int argc, char * argv[]){
	

	// define the ros node
	ros::init(argc,argv, "joint");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");
	double roll, pitch, yaw, x, y, z;
	home.getParam("needle_length",needle_length);
	// when not manually commanding the joints, xyz-rpy 
	// coordinates are read from the launch file to use 

	// define the kinematic chain
	KDL::Chain chain = LWR();
	// define the forward kinematic solver via the defined chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	// define the inverse kinematics solver
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6

	// get the number of joints from the chain
	unsigned int nj = chain.getNrOfJoints();
	// define a joint array in KDL format for the joint positions
   	KDL::JntArray jointpositions = KDL::JntArray(nj);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	// define a manual joint command array for debugging	
	KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);
	
	for (int i = 0; i < nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "j";
		joint_name << i+1;
		home.getParam(joint_name.str(),manual_joint_cmd(i));
	}


	// setting up the loop frequency 
	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	
	
	ros::Publisher xyzrpy_pub = nh_.advertise<geometry_msgs::Twist>("/robot_tip_pose",10);


	// subscriber for reading the joint angles from the gazebo simulator
	ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states",10, get_joints);
	
	

	


	KDL::Frame cartpos;    
	KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw); //Rotation built from Roll-Pitch-Yaw angles
 
	// for debugging: Calculate forward position kinematics
	bool kinematics_status;

	// debugging variables
	geometry_msgs::Twist xyz;



	while(ros::ok()){		
		if (initialized){
			// update the joint positions with the most recent readings from the joints
			for (int k = 0; k<7; ++k){
				jointpositions(k) = joints.position[k];
			}				


			kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
			if(kinematics_status>=0){
				xyz.linear.x = cartpos.p[0];
				xyz.linear.y = cartpos.p[1];
				xyz.linear.z = cartpos.p[2];
				cartpos.M.GetRPY(roll,pitch, yaw);
				xyz.angular.x = roll;
				xyz.angular.y = pitch;
				xyz.angular.z = yaw;
			}
			xyzrpy_pub.publish(xyz);			
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

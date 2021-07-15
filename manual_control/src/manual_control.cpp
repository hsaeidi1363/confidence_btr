/*
 * This code was developed for the manual and semi-autonomous control modes of the confidence-based control strategy between 2019-2021
 * The code reads the initial position of the kuka arm and uses it as a reference for sending X-Y-Z control commands
 * from the readings of the haptic device positions. It also applies a force feedback on the Z axis to keep a correct height for
 * device during the tests. Once the robot is switched to the autonomous mode, an X-Y-Z force feedback controls the position of the haptic
 * device to follow and match the robot positions. This is important because once the operator returns back to the loop via the manual mode,
 * they should continue controlling the robot from the same positions that the autonomous controller was de-activated. 
 * 
 */

#include<ros/ros.h>
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<tf/transform_listener.h>
#include<iiwa_msgs/TimeToDestination.h>
#include<iiwa_msgs/JointPosition.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>



double needle_length = 0.0;

// defining the kinematic chain of robot for sending the control commands to the robot
//Frame KDL::Frame::DH_Craig1989 (double a, double alpha, double d, double theta)
KDL::Chain LWR(){

  double tool_length = 0.37826;
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
sensor_msgs::JointState joints_kuka;

bool initialized_kuka = false;

//callback for reading joint values
void get_joints_kuka(const sensor_msgs::JointState & data){
	for (int i = 0; i < data.position.size();++i){
		// if this is not the first time the callback function is read, obtain the joint positions
		if(initialized_kuka){
			joints_kuka.position[i] = data.position[i];
		}
	}	
	initialized_kuka = true;
}


// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions(i) = _init;
}

void initialize_joints(sensor_msgs::JointState & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions.position.push_back(_init);
}


// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}


//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj){
	for (int i = 1; i <= _nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "iiwa_joint_";
		joint_name << i;
		_cmd.joint_names.push_back(joint_name.str());
	}
}


void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj){
	for (int i = 0; i < _nj; ++i){
		 while(_jointpositions(i) > M_PI)
				_jointpositions(i) -= 2*M_PI;
		 while(_jointpositions(i) < -M_PI)
				_jointpositions(i) += 2*M_PI;
		_point.positions[i] = _jointpositions(i);
	}	
}

// a quick code for checking the smoothness of commands
void convert_to_fast(iiwa_msgs::JointPosition & _fast_cmd, trajectory_msgs::JointTrajectoryPoint & _point){
	_fast_cmd.position.a1 = _point.positions[0];
	_fast_cmd.position.a2 = _point.positions[1];
	_fast_cmd.position.a3 = _point.positions[2];
	_fast_cmd.position.a4 = _point.positions[3];
	_fast_cmd.position.a5 = _point.positions[4];
	_fast_cmd.position.a6 = _point.positions[5];
	_fast_cmd.position.a7 = _point.positions[6];
}

// autonomous control command variable
//trajectory_msgs::JointTrajectory auto_cmd;
//new command 
iiwa_msgs::JointPosition auto_cmd;


// current and previous centering forces for the haptic device
omni_msgs::OmniFeedback centering_force;
omni_msgs::OmniFeedback centering_force_prev;


// current and previous readings of the haptic device buttons
omni_msgs::OmniButtonEvent button;
omni_msgs::OmniButtonEvent prev_button;

//debugging variable
geometry_msgs::Twist dbg;

std_msgs::Bool test_start;

bool autonomous_mode = false; // manual is 1 and 0 is auto

//read the autonomous control command
//void get_auto(const trajectory_msgs::JointTrajectory & _data){
void get_auto(const iiwa_msgs::JointPosition & _data){
	auto_cmd = _data;
}

geometry_msgs::PoseStamped phantom_pos;

void get_phantom_pos(const geometry_msgs::PoseStamped & _data){
	phantom_pos = _data;

}


// variable for tracking if the reference position of the haptic devices has changed
bool start_loc_available = false;
std_msgs::Bool reset_auto;

void get_button(const omni_msgs::OmniButtonEvent & _data){
	button = _data;
        // check if the control mode has changed via the grey button
        if(button.grey_button == 1 && prev_button.grey_button == 0){
		autonomous_mode = !autonomous_mode;
		start_loc_available = autonomous_mode;//when switching to manual, start location has to be rest to the current pose
		reset_auto.data = autonomous_mode;
	}		
        // check if the test has started via the white button
	if(button.white_button == 1 && prev_button.white_button == 0){
		test_start.data = !test_start.data;
	}		
	prev_button = button;
}




// desired values of x, y, z for centering the haptic device
double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;


// proportional and derivate control gains for the x,y,z axis of the haptic device when tracking a certain reference position of the robot or centering them
double kp_x = 25.0;
double kd_x = 20.0;
double kp_y = 25.0;
double kd_y = 20.0;
double kp_z = 50.0;
double kd_z = 20.0;


double e_x_prev = 0.0, e_y_prev = 0.0, e_z_prev = 0.0;

//PD tracker for the position of the haptic device
void calc_center_force(void){
	double e_x, e_y, e_z, de_x, de_y, de_z;
	//calculate the error
        e_x = x_d - phantom_pos.pose.position.x; 
	e_y = y_d - phantom_pos.pose.position.y; 
	e_z = z_d - phantom_pos.pose.position.z; 
        //calculate the derivatives of the errors
        de_x = e_x - e_x_prev;
	de_y = e_y - e_y_prev;
	de_z = e_z - e_z_prev;
        //low pass filter for reducing the noise and jerks in the forces
	double tau = 0.8;
	centering_force.force.x = (kp_x*e_x + kd_x*de_x)*(1-tau) + tau*centering_force_prev.force.x;
	centering_force.force.y = (kp_y*e_y + kd_y*de_y)*(1-tau) + tau*centering_force_prev.force.y;
     	centering_force.force.z = (kp_z*e_z + kd_z*de_z)*(1-tau) + tau*centering_force_prev.force.z + 0.2;//the last component is for the effect hand weight
	// reduce the kicks by resetting th force feedbacks when the reference changes
 	e_x_prev = e_x;
	e_y_prev = e_y;
	e_z_prev = e_z;
	centering_force_prev = centering_force;
}


int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

double dead_zone(double _val){
	double d = 0.0075; //deadzone value of 1cm
	if (fabs(_val) <= d){
		return 0.0;
	}else{
		return _val - sgn(_val)*d;  
	}
	
}
void convert_commands(geometry_msgs::Twist & _cmd){
	double c_gain = 0.175;
	// xc = xh, yc = -zh, zc = -yh
	_cmd.linear.x = dead_zone(phantom_pos.pose.position.x)*c_gain;
	_cmd.linear.y = -dead_zone(phantom_pos.pose.position.z)*c_gain*1.3;
	_cmd.linear.z = dead_zone(phantom_pos.pose.position.y)*c_gain*2;
}


int main(int argc, char * argv[]){

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

	ros::init(argc, argv, "manual_control");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	bool semi_auto = false;
	home.getParam("semi_auto",semi_auto);

	int loop_freq = 10;
        float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	ros::Publisher force_pub =nh_.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback",1);
        
        // defining the puilsher that accepts joint position commands and applies them to the simulator or real robot
	//std::string command_topic = "iiwa/PositionJointInterface_trajectory_controller/command";
	std::string command_topic = "/iiwa/command/JointPosition";

	//ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);

	ros::Publisher cmd_pub = nh_.advertise<iiwa_msgs::JointPosition>(command_topic,1);


	ros::Publisher control_mode_pub = nh_.advertise<std_msgs::UInt8>("iiwa/control_mode",1);

	ros::Publisher test_start_pub = nh_.advertise<std_msgs::Bool>("/test_start",10);

	ros::Publisher reset_auto_pub = nh_.advertise<std_msgs::Bool>("/reset_auto_traj",10);
     
       	ros::Publisher dbg_pub = nh_.advertise<geometry_msgs::Twist>("/hapticdbg",10);

	// subscriber for reading the joint angles from the gazebo simulator
        ros::Subscriber joints_kuka_sub = nh_.subscribe("/iiwa/joint_states",10, get_joints_kuka);


        ros::Subscriber auto_sub = nh_.subscribe("/iiwa/auto/command",10, get_auto);
        

	ros::Subscriber pos_sub = nh_.subscribe("/phantom/pose",10, get_phantom_pos);
	
	ros::Subscriber button_sub = nh_.subscribe("/phantom/button",10, get_button);

	ros::ServiceClient client = nh_.serviceClient<iiwa_msgs::TimeToDestination>("iiwa/state/timeToDestination");
	iiwa_msgs::TimeToDestination config;

        trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint pt;
	iiwa_msgs::JointPosition fast_joint_cmd;

	initialize_points(pt,nj,0.0);
	// define the joint names, e.g. iiwa_joint_1 up to iiwa_joint_7
	name_joints(joint_cmd, nj);

	//kdl version
	initialize_joints(jointpositions, nj, 0.2);
	//sensor_msgs version
        initialize_joints(joints_kuka, nj, 0.0);

	KDL::Frame cartpos;
	double roll, pitch, yaw;
	KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw); //Rotation built from Roll-Pitch-Yaw angles
	joint_cmd.points.push_back(pt);
	//new reference position for KUKA in world
        geometry_msgs::Twist ref_in_kuka;
	//initial 3D pose of KUKA in world
        geometry_msgs::Twist kuka_initial;
	// delta position in the camera frame
        geometry_msgs::Twist command_c;

	// for debugging: Calculate forward position kinematics
        bool kinematics_status;

        bool all_zero = true;

	std_msgs::UInt8 control_mode;
        test_start.data = false;

        tf::TransformListener listener;
        tf::StampedTransform transform;
        tf::Transform inv_transform;


        while (ros::ok()){
               
                try{
                    listener.lookupTransform("flea", "kuka", ros::Time(0), transform);
                    inv_transform = transform.inverse();
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
		tf::Vector3 pos_in_camera, pos_in_world;
		if (initialized_kuka){
			
			//update the kdl variables based on the joint readings from robot
			for (int k = 0; k<7; ++k){
				jointpositions(k) = joints_kuka.position[k];
                                all_zero &= (fabs(jointpositions(k)) <.01);
                                    
			}
		}
		if(!all_zero){
                        //find where the robot is initially
			if(!start_loc_available){
                                kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
                                if(kinematics_status>=0){
                                        kuka_initial.linear.x = cartpos.p[0];
                                        kuka_initial.linear.y = cartpos.p[1];
                                        kuka_initial.linear.z = cartpos.p[2];
                                        cartpos.M.GetRPY(roll,pitch, yaw);
                                        kuka_initial.angular.x = roll;
                                        kuka_initial.angular.y = pitch;
                                        kuka_initial.angular.z = yaw;
                                }
                                start_loc_available = true;
                               
                        }else{
				//convert from the haptic device frames to camera frame with additional deadzone and scaling
				convert_commands(command_c);
				dbg  = command_c;

				pos_in_camera = 	transform(tf::Vector3(kuka_initial.linear.x,kuka_initial.linear.y,kuka_initial.linear.z));
				dbg.angular.x = pos_in_camera.x();
				dbg.angular.y = pos_in_camera.y();
				dbg.angular.z = pos_in_camera.z();

				geometry_msgs::Twist tmp;
				tmp.linear.x = pos_in_camera.x() + command_c.linear.x;
				tmp.linear.y = pos_in_camera.y() + command_c.linear.y;
				tmp.linear.z = pos_in_camera.z() + command_c.linear.z;

				pos_in_world = inv_transform(tf::Vector3(tmp.linear.x,tmp.linear.y,tmp.linear.z));

				ref_in_kuka.linear.x = pos_in_world.x();
				ref_in_kuka.linear.y = pos_in_world.y();
				ref_in_kuka.linear.z = pos_in_world.z();
				cartpos.p[0] = ref_in_kuka.linear.x;
				cartpos.p[1] = ref_in_kuka.linear.y;
				cartpos.p[2] = ref_in_kuka.linear.z;

				int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
                                eval_points(pt, jointpositions_new, nj);
                                pt.time_from_start = ros::Duration(dt);
                                joint_cmd.points[0] = pt;
				convert_to_fast(fast_joint_cmd, pt);
			

				if (reset_auto.data){
					reset_auto_pub.publish(reset_auto);
					reset_auto.data = false;
				}else if (autonomous_mode){
					float rem_time;
					/*if(client.call(config)){
						rem_time = config.response.remaining_time;
						if(rem_time < 0.0){
							reset_auto.data = true;
							reset_auto_pub.publish(reset_auto);
							reset_auto.data = false;
							std::cout << "reset the traj"<<std::endl;
						}
					//	std::cout << "got the service value "<< rem_time<<std::endl;
					}else{
						std::cout << "could not call the service"<<std::endl;
					}
					*/
					//joint_cmd = auto_cmd;
					fast_joint_cmd.position = auto_cmd.position;                                
        	                }
				//joint_cmd.header.stamp = ros::Time::now();
                                //cmd_pub.publish(joint_cmd);
				fast_joint_cmd.header.stamp = ros::Time::now();
				cmd_pub.publish(fast_joint_cmd);
				
				//dbg.linear.x = tmp.linear.x;
				//dbg.linear.y = tmp.linear.y; 
				//dbg.linear.z = tmp.linear.z;
				dbg_pub.publish(dbg);
				
			}
			//end of if(!start_loc_available)


		}

		if(autonomous_mode)
			control_mode.data = 0;
		else
			control_mode.data = 1;

		control_mode_pub.publish(control_mode);

		calc_center_force();
		force_pub.publish(centering_force);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

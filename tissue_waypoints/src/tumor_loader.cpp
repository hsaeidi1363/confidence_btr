#include <ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<iostream>
#include <std_msgs/Bool.h>
#include<geometry_msgs/PolygonStamped.h>


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
  ros::init( argc, argv, "tumor_loader" );

  ros::NodeHandle nh, nhp("~");
  ros::Rate loop_rate(5);
  ros::Subscriber sub_send_plan = nh.subscribe("/send_plan", 1, update_plan);
  ros::Publisher  tumor_pub =  nh.advertise<geometry_msgs::PolygonStamped>( "/offline_tumor", 1 );
	
  
  
  nhp.getParam("plan_file_name", plan_file_name);
  plan_file_name = ros::package::getPath("tissue_waypoints") + plan_file_name;
  ifstream plan_file(plan_file_name.c_str(), ifstream::in);
  check_files(plan_file,plan_file_name);
  string line;
  int ctr = 1;
  geometry_msgs::PolygonStamped polygon;


  while(getline(plan_file, line)){
    istringstream iss(line);
    // read the positions
	geometry_msgs::Point32 p;
    float x_tmp;
    float y_tmp;
    iss >> x_tmp;
    iss >> y_tmp;
    p.x = (int) x_tmp;
    p.y = (int) y_tmp;
    p.z = 0; 
    polygon.polygon.points.push_back( p );

    cout << "read coordinates for point: " << ctr<<" at: x= "<< p.x  << " (pixels) y= "<<p.y  << " (pixels) and z = "<< p.z  << " (pixels)"<< endl;
    ctr ++;
  }


	   
		

  
  int seq = 0;
  while(ros::ok()){
	if(send_plan){
		  polygon.header.stamp = ros::Time::now();
	      tumor_pub.publish(polygon);
	     // send_plan = false;
	     // std::cout << "sent a plan \n";
	}
	loop_rate.sleep();
	ros::spinOnce();
  }
  plan_file.close();
  return 0;

}

#include<ros/ros.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include<tf_conversions/tf_eigen.h>
#include<geometry_msgs/Twist.h>

// TODO: plan points should be in the robot frame instead of camera frame
// Find the closest point of the plan in the robot frame and then use it

using namespace std;
class Planner{
  public:
    ros::NodeHandle nh_;
    Planner(ros::NodeHandle nh):
      nh_(nh){
        waypoint_sub = nh_.subscribe("filtered_tissue_traj",1,&Planner::get_traj, this);
        robot_pos_sub = nh_.subscribe("robot/worldpos", 1, &Planner::get_rob_pos, this);
        plan_pub =  nh_.advertise<trajectory_msgs::JointTrajectory>("/plan",1);
	dbg_traj_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("plancloud",1);
      }
    void get_traj(const sensor_msgs::PointCloud2Ptr & _cloud){   
        pcl::fromROSMsg(*_cloud,traj_cloud);
        if(rob_pos_available){
         transform_to_robot(traj_cloud); 
        }
    }
    void get_rob_pos(const geometry_msgs::Twist & _data){
      rob_pos = _data;
      rob_pos_available = true;
    } 
    // convert the waypoint point cloud to some waypoints in the robot frame
    void transform_to_robot(pcl::PointCloud<pcl::PointXYZ> &_in){
      try{
        // TODO: make sure when using the actual robot the frames match 
        tf::StampedTransform cam_in_rob;//transformation of intel camera in the world frame
        // get the transformation
//        listener.lookupTransform("world", "intel", ros::Time(0), cam_in_rob);
        listener.lookupTransform("kuka", "intel", ros::Time(0), cam_in_rob);
        tf::Transform inv_transform;
        inv_transform = cam_in_rob.inverse();

        // this is just for tranforming the waypoints to robot frame but later we need to find out which one is the closest and sort the points sequentially for the controller
        trajectory_msgs::JointTrajectory unsorted_plan;
        // loop through the waypoints and convert to the robot frame
        pcl::PointCloud<pcl::PointXYZ>::iterator ctr = _in.begin(); 
        for (; ctr != _in.end(); ctr++){
          tf::Vector3 pose_in_world;
          // convert to robot frame
          pose_in_world = cam_in_rob(tf::Vector3(ctr->x, ctr->y, ctr->z)); 
          trajectory_msgs::JointTrajectoryPoint plan_pt; 
 
          plan_pt.positions.push_back(pose_in_world.x()); 
          plan_pt.positions.push_back(pose_in_world.y()); 
          plan_pt.positions.push_back(pose_in_world.z()+ 0.002); 

          plan_pt.positions.push_back(rob_pos.angular.x);
          plan_pt.positions.push_back(rob_pos.angular.y);
          plan_pt.positions.push_back(rob_pos.angular.z);
          for (int ind = 0; ind < 6; ind ++){
            plan_pt.velocities.push_back(0);
            plan_pt.accelerations.push_back(0);
          }
          unsorted_plan.points.push_back(plan_pt);
        }// now we have all the unsorted waypoints in the robot frame
        int waypoints_len = unsorted_plan.points.size();
        ROS_INFO("we have %d unsorted waypoints",waypoints_len);
        int min_ind = 0;
        float dist = 1000.0;
        for (int i = 0; i < waypoints_len; i++){
          float dist_tmp;
          dist_tmp = calc_dist(unsorted_plan.points[i],rob_pos); 
          if (dist_tmp < dist){
            min_ind = i;
            dist = dist_tmp;
          }
        }
	ROS_INFO("min dist is now %f",(float) dist);
	if (dist > 0.015 || dist < 0.002){
		plan.points.clear();
		dbg_cloud.points.clear();
		ROS_INFO("cleared the old sorted plan and the length is now %d",(int) plan.points.size());
		for (int i = 0; i < 3; i++){
		  int ind = (i + min_ind) % waypoints_len;
		  plan.points.push_back(unsorted_plan.points[ind]);
		  tf::Vector3 pose_in_cam;
		  // convert to robot frame
		  pose_in_cam = inv_transform(tf::Vector3(unsorted_plan.points[ind].positions[0], unsorted_plan.points[ind].positions[1], unsorted_plan.points[ind].positions[2])); 
		  pcl::PointXYZI tmp;
		  tmp.x = pose_in_cam.x();
		  tmp.y = pose_in_cam.y();
	  	  tmp.z = pose_in_cam.z();
		  tmp.intensity = 1;
		  dbg_cloud.points.push_back(tmp);
		}
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("camera_color_optical_frame");
		dbg_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_pub.publish(dbg_cloud);
		plan_pub.publish(plan);
	}
      }     
      catch(...){
        cout << "frames not read correctly"<< endl;
      }
    }
    float calc_dist(trajectory_msgs::JointTrajectoryPoint & _pt, geometry_msgs::Twist & _rob_pos){
      float dx = _pt.positions[0] - _rob_pos.linear.x;
      float dy = _pt.positions[1] - _rob_pos.linear.y;
      float dz = _pt.positions[2] - _rob_pos.linear.z;
      return sqrt(dx*dx + dy*dy + dz*dz);
    }
  private:
    ros::Subscriber waypoint_sub;
    ros::Subscriber robot_pos_sub;
    ros::Publisher plan_pub;
    ros::Publisher dbg_traj_pub;
    tf::TransformListener listener;
    trajectory_msgs::JointTrajectory plan;
    pcl::PointCloud<pcl::PointXYZ> traj_cloud;
    pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
    geometry_msgs::Twist rob_pos;
    bool rob_pos_available;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  Planner planner(n);
  ros::spin();
  return 0;
}

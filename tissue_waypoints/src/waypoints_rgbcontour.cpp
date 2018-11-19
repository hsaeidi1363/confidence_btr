#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<sensor_msgs/Image.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#define PI 3.14159 

using namespace cv;
using namespace std;
using namespace cv_bridge;


// raw input image
Mat img;
// pointer for reading ros images via cv brdige
CvImagePtr cv_ptr;


// canny edge detection threshold
int thresh_low = 30;
int thresh_high = 220;


// initialization check to see if the first image is received or not
bool initialized = false;

// max x and y coordinates take from the camera
double width, height;

vector<Point2f> image_corners;
	
// callback function for the input image
void get_img(const sensor_msgs::Image &  _data){
	// read the rectified rgb input image from the camera
	cv_ptr = toCvCopy(_data,"rgb8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;
	
	if (!initialized){
		image_corners.push_back(Point2f(0.0f, height));
		image_corners.push_back(Point2f(width, height));
		image_corners.push_back(Point2f(0.0f,0.0f));
		image_corners.push_back(Point2f(width, 0.0f));
	}
	initialized = true;
}

pcl::PointCloud<pcl::PointXYZ> pointcloud_in;
void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud) {

  pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud

}

// find the distance between to XY points
double distance (double x1, double y1, double x2, double y2){
	double dx = x1-x2;
	double dy = y1-y2;
	return (sqrt(dx*dx+dy*dy));
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"waypoint_rgbcontour");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

    vector<vector<Point> > contours;

    bool contour_detection = false;
	bool partial_trajectory = false;
    bool track_detected = false;


    int track_ctr = 0;
	// number of waypoints sent to the robot
	int npt = 1;
	int roi_l = 0;
	int roi_r = 0;
	int roi_u = 0;
	int roi_b = 0;
	//TODO: set the params in the launch file
	home.getParam("contour", contour_detection);	
	home.getParam("partial_traj", partial_trajectory);
	home.getParam("number_of_waypoints", npt);	
	home.getParam("roi_l", roi_l);	
	home.getParam("roi_r", roi_r);
	home.getParam("roi_u", roi_u);
	home.getParam("roi_b", roi_b);
	
	// set the loop frequency equal to the camera input
	//TODO: set the correct frequency based on the RGBD camera
	int loop_freq = 7;
	ros::Rate loop_rate(loop_freq);
	ros::Subscriber cam_sub = nh_.subscribe("camera/image_rect_color",2,get_img);

    ros::Subscriber pcl_sub = nh_.subscribe("camera/depth_registered/points", 10, get_pcl);


	return 0;
}
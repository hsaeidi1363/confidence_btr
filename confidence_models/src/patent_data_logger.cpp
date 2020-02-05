// this codes overlays the 2D position of the NIR markers on the RGB image of intel camera (need to run the nir_overlay.launch before this)
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include<std_msgs/Bool.h>

#include <string>



using namespace cv;
using namespace std;
using namespace cv_bridge;


bool freeze_path = false;
bool go_to_next = false;

void get_freeze(const std_msgs::Bool & _data){
	freeze_path = _data.data;	
}

void get_next(const std_msgs::Bool & _data){
	go_to_next = _data.data;	
}


// raw input image
Mat img;
// pointer for reading ros images via cv brdige
CvImagePtr cv_ptr;
// region of interest for cropping the camera view for the operator
cv::Rect roi;
int roi_l = 0;
int roi_r = 0;
int roi_u = 0;
int roi_b = 0;

// initialization check to see if the first image is received or not
bool initialized = false;
bool caminfo_received = false;

// max x and y coordinates take from the camera
double width, height;

// callback function for the input image
void get_img(const sensor_msgs::Image &  _data){
	// read the rectified rgb input image from the camera
	cv_ptr = toCvCopy(_data,"rgb8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;


	roi.x = roi_l;
   	roi.y = roi_u;
	roi.width = width - roi_r - roi_l;
	roi.height = height - roi_b - roi_u;

	
	initialized = true;
}


      
cv::Mat K_intel = cv::Mat::zeros( 3, 3, CV_64FC1 );       // camera matrix  
cv::Mat D_intel = cv::Mat::zeros( 1, 5, CV_64FC1 );       // camera distortion
      

void get_caminfo(const sensor_msgs::CameraInfo& msgintelinfo){
    K_intel.at<double>( 0, 0 ) = msgintelinfo.K[0];
    K_intel.at<double>( 0, 2 ) = msgintelinfo.K[2];
    K_intel.at<double>( 1, 1 ) = msgintelinfo.K[4];
    K_intel.at<double>( 1, 2 ) = msgintelinfo.K[5];
    K_intel.at<double>( 2, 2 ) = 1.0;
    for( size_t i=0; i<5; i++ ) { D_intel.at<double>( i ) = msgintelinfo.D[i]; }
    caminfo_received = true;
   // cout << "Received camera info" << " " <<K_intel.at<double>( 0, 0 ) << " " <<K_intel.at<double>( 0, 2) << " " <<K_intel.at<double>( 1, 1 )<< " " <<K_intel.at<double>( 1, 2 )<< " " <<K_intel.at<double>( 2, 2 ) << endl;
   // cout << " " <<D_intel.at<double>(0)<< " " <<D_intel.at<double>(1)<< " " <<D_intel.at<double>(2)<< " " <<D_intel.at<double>(3)<< " " <<D_intel.at<double>(4)<< endl;
}


geometry_msgs::Twist rob_pos;

void get_robot_pos(const geometry_msgs::Twist & _data){

	rob_pos = _data;	
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"identification_manual");
	ros::NodeHandle nh_;

	ros::NodeHandle home("~");
	
	home.getParam("roi_l", roi_l);	
	home.getParam("roi_r", roi_r);
	home.getParam("roi_u", roi_u);
	home.getParam("roi_b", roi_b);

	std::string test_no = "dbg";
	home.getParam("test_no", test_no);

	bool show_markers = false;
	home.getParam("show_markers", show_markers);


    	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);

	// subscribe to the rectified rgb input (needs the calibration file read in the launch file for the camera)
	ros::Subscriber cam_sub = nh_.subscribe("/flea/camera/image_color",1,get_img);
	ros::Subscriber caminfo_sub = nh_.subscribe("flea/camera/camera_info",1,get_caminfo);
	ros::Subscriber freeze_path_sub = nh_.subscribe("/freeze_path",1,get_freeze);	
	ros::Subscriber go_to_next_sub = nh_.subscribe("/go_to_next",1,get_next);	

	ros::Subscriber rob_pos_sub = nh_.subscribe("/robot/worldpos",1,get_robot_pos);	
	
	//publisher for checking the images and debugging them
	ros::Publisher overlay_pub = nh_.advertise<sensor_msgs::Image>("niroverlay_2D",1);

	tf::TransformListener listener;



	// translation and rotations for finding the 2D location of NIR markers on the RGB image
   	cv::Mat cvt_intel = cv::Mat::zeros( 3, 1, CV_64FC1 );; // translation
   	cv::Mat cvR_intel = cv::Mat::zeros( 3, 3, CV_64FC1 );; // rotation matrix
   	cvR_intel.at<double>( 0, 0 ) = 1;
   	cvR_intel.at<double>( 1, 1 ) = 1;
   	cvR_intel.at<double>( 2, 2 ) = 1;

	std::vector< cv::Point2f > stduv_old;
    	std::vector< cv::Point3f > stdxyz;

	std::vector< tf::Vector3 > pattern_pts;

	pattern_pts.push_back(tf::Vector3(0.031734, 0.0476, 0.0));
	pattern_pts.push_back(tf::Vector3(0.031734, 0.0, 0.0));
	pattern_pts.push_back(tf::Vector3(0.0, 0.0476, 0.0));
	pattern_pts.push_back(tf::Vector3( 0.0, 0.0, 0.0));
	int order[13] = {0, 1, 2, 3, 0, 2, 1, 3, 2, 0, 3, 1, 0};


    	//stdxyz.push_back( cv::Point3f ( 0.0, 0.0, 0.0) );
    	//stdxyz.push_back( cv::Point3f ( 0.0, 0.042312, 0.0) );
    	//stdxyz.push_back( cv::Point3f ( 0.031734, 0.042312, 0.0) );
    	//stdxyz.push_back( cv::Point3f ( 0.031734, 0.0, 0.0) );
	int ctr = 0;
	std::ofstream positions_file;
	positions_file.open(("/home/hsaeidi/ident_data/"+test_no+"_positions.csv").c_str());
	// should I add haptic device inputs as well?! -----or the commands coming to the robot --------
	positions_file << "p1_x,p1_y,p1_z,p2_x,p2_y,p2_z,x,y,z,r,p,y" << std::endl;

	while(ros::ok()){
		if (initialized && caminfo_received ){ 
    		  std::vector< cv::Point2f > stduv;
		  if(freeze_path){
			stduv = stduv_old;
		  }else if(pattern_pts.size() != 0){
			tf::StampedTransform tfRt;
      			listener.lookupTransform( "flea", "pattern", ros::Time(0), tfRt );

			if (go_to_next){
				ctr ++;
				go_to_next = false;
				ROS_INFO("Switched to path #: %d", ctr);
			}
			stdxyz.clear();
			tf::Vector3 start_pt;
			tf::Vector3 end_pt;

			start_pt = tfRt(pattern_pts[order[ctr - 1]]);
			stdxyz.push_back( cv::Point3f (start_pt[0], start_pt[1], start_pt[2]));

			end_pt = tfRt(pattern_pts[order[ctr]]);
			stdxyz.push_back( cv::Point3f (end_pt[0], end_pt[1], end_pt[2]));

			if(ctr > 0){

				tf::StampedTransform flea_in_kuka;
				listener.lookupTransform("kuka", "flea", ros::Time(0), flea_in_kuka);

				start_pt = flea_in_kuka(start_pt);		
				end_pt = flea_in_kuka(end_pt);
				positions_file << start_pt[0]<<","<< start_pt[1]<<","<< start_pt[2]<<","<< end_pt[0]<<","<< end_pt[1]<<","<< end_pt[2]<<","<<rob_pos.linear.x <<","<<rob_pos.linear.y <<","<<rob_pos.linear.z <<","<<rob_pos.angular.x<<","<<rob_pos.angular.y<<","<<rob_pos.angular.z<<endl ;
			}
			
/*
 			for (int i = 0; i < pattern_pts.size(); i++){
				tf::Vector3 tmp;
				tmp = tfRt(pattern_pts[i]);
				stdxyz.push_back( cv::Point3f (tmp[0], tmp[1], tmp[2]));
			}
*/			
      			Eigen::Affine3d eigRt;
    			tf::transformTFToEigen( tfRt, eigRt );

 
			cv::projectPoints(stdxyz , cvR_intel, cvt_intel, K_intel, D_intel, stduv );
			stduv_old.clear();
			stduv_old = stduv;
		  }
		  if (stduv.size() == 2){
			if (show_markers){		
					circle(img, Point(stduv[0].x, stduv[0].y), 3, Scalar(0,255,0), 2, LINE_AA);
					circle(img, Point(stduv[1].x, stduv[1].y), 3, Scalar(0,0,255), 2, LINE_AA);
			}

		  }else{
			  for (int i = 0; i < stduv.size(); i++){		  
				if (show_markers)		
					circle(img, Point(stduv[i].x, stduv[i].y), 3, Scalar(0,255,0), 2, LINE_AA);
			  }
		  }
	   	  Mat img_crop = img(roi);
		  cv_ptr->image = img_crop;
        	  overlay_pub.publish(cv_ptr->toImageMsg());

		}
		if (ctr > 12){
			ROS_INFO("Finished all the paths");
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	positions_file.close();
	return 0;
	
}

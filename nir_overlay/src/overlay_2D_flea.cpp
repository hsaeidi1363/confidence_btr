// this codes overlays the 2D position of the NIR markers on the RGB image of flea camera (need to run the nir_overlay.launch before this)
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami

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
#include<std_msgs/UInt8.h>
#include<geometry_msgs/Twist.h>


using namespace cv;
using namespace std;
using namespace cv_bridge;

int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

bool freeze_path = false;

void get_freeze(const std_msgs::Bool & _data){
	freeze_path = _data.data;	
}

geometry_msgs::Twist teleop_cmd; 
void get_teleop_commands(const geometry_msgs::Twist & _data){
	teleop_cmd = _data;	
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
bool pcl_received = false;

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


      
cv::Mat K_flea = cv::Mat::zeros( 3, 3, CV_64FC1 );       // camera matrix  
cv::Mat D_flea = cv::Mat::zeros( 1, 5, CV_64FC1 );       // camera distortion
      

void get_caminfo(const sensor_msgs::CameraInfo& msgfleainfo){
    K_flea.at<double>( 0, 0 ) = msgfleainfo.K[0];
    K_flea.at<double>( 0, 2 ) = msgfleainfo.K[2];
    K_flea.at<double>( 1, 1 ) = msgfleainfo.K[4];
    K_flea.at<double>( 1, 2 ) = msgfleainfo.K[5];
    K_flea.at<double>( 2, 2 ) = 1.0;
    for( size_t i=0; i<5; i++ ) { D_flea.at<double>( i ) = msgfleainfo.D[i]; }
    caminfo_received = true;
}

pcl::PointCloud<pcl::PointXYZI> marker_cog_pcl;
pcl::PointCloud<pcl::PointXYZI> gui_pcl;
std::vector< cv::Point3f > stdxyz;
std::vector< cv::Point3f > stdxyz_gui;


void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){
	pcl::fromROSMsg(*cloud, marker_cog_pcl);

    pcl::PointCloud<pcl::PointXYZI>::iterator pi=marker_cog_pcl.begin();

    stdxyz.clear();
      //for( ; pi!=pclxyzi.end(); pi++, prgb++ ) {
	for( ; pi!=marker_cog_pcl.end(); pi++ ) {
	    if( 0 < fabs( pi->x ) && 0 < fabs( pi->y ) && 0 < fabs( pi->z ) ){
			 stdxyz.push_back( cv::Point3f ( pi->x, pi->y, pi->z ) ); 
		}

    }
	pcl_received = true;
}


void get_gui_points(const sensor_msgs::PointCloud2Ptr& cloud){
    pcl::fromROSMsg(*cloud, gui_pcl);
    pcl::PointCloud<pcl::PointXYZI>::iterator pi=gui_pcl.begin();

    stdxyz_gui.clear();
      //for( ; pi!=pclxyzi.end(); pi++, prgb++ ) {
     for( ; pi!=gui_pcl.end(); pi++ ) {
	    if( 0 < fabs( pi->x ) && 0 < fabs( pi->y ) && 0 < fabs( pi->z ) ){
			 stdxyz_gui.push_back( cv::Point3f ( pi->x, pi->y, pi->z ) ); 
	     }

    }
}

bool tele_op_mode = true;

void get_control_mode(const std_msgs::UInt8 & _data){
	if(_data.data == 1){
		tele_op_mode = true;
	}
	if(_data.data == 0){
		tele_op_mode = false;
	}
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"overlay_2D");
	ros::NodeHandle nh_;

	ros::NodeHandle home("~");
	
	home.getParam("roi_l", roi_l);	
	home.getParam("roi_r", roi_r);
	home.getParam("roi_u", roi_u);
	home.getParam("roi_b", roi_b);

	bool show_markers = false;
	home.getParam("show_markers", show_markers);


        int loop_freq = 25;
	ros::Rate loop_rate(loop_freq);

	// subscribe to the rectified rgb input (needs the calibration file read in the launch file for the camera)
	ros::Subscriber cam_sub = nh_.subscribe("/flea/camera/image_color",1,get_img);
	ros::Subscriber caminfo_sub = nh_.subscribe("flea/camera/camera_info",1,get_caminfo);
	ros::Subscriber pcl_sub = nh_.subscribe("/path_confidence",1,get_pcl);
	ros::Subscriber gui_points_sub = nh_.subscribe("/user_gui",1,get_gui_points);
	ros::Subscriber freeze_path_sub = nh_.subscribe("/freeze_path",1,get_freeze);	
	ros::Subscriber control_mode_sub = nh_.subscribe("/iiwa/control_mode",1,get_control_mode);	
	ros::Subscriber teleop_commands_sub = nh_.subscribe("/hapticdbg",1,get_teleop_commands);	

	
	//publisher for checking the images and debugging them
	ros::Publisher overlay_pub = nh_.advertise<sensor_msgs::Image>("niroverlay_2D",1);

	tf::TransformListener listener;



	// translation and rotations for finding the 2D location of NIR markers on the RGB image
    	cv::Mat cvt_flea = cv::Mat::zeros( 3, 1, CV_64FC1 );; // translation
    	cv::Mat cvR_flea = cv::Mat::zeros( 3, 3, CV_64FC1 );; // rotation matrix
    	cvR_flea.at<double>( 0, 0 ) = 1;
    	cvR_flea.at<double>( 1, 1 ) = 1;
    	cvR_flea.at<double>( 2, 2 ) = 1;
	std::vector< cv::Point2f > stduv_old;
	while(ros::ok()){
		if (initialized && caminfo_received && pcl_received){ 
	    	  std::vector< cv::Point2f > stduv;
	    	  std::vector< cv::Point2f > stduv_gui;
		  tf::StampedTransform tfRt;
		  listener.lookupTransform( "flea", "camera_depth_optical_frame", ros::Time(0), tfRt );

		  Eigen::Affine3d eigRt;
		  tf::transformTFToEigen( tfRt, eigRt );
 		  cvt_flea.at<double>( 0, 0 ) = tfRt.getOrigin()[0];
		  cvt_flea.at<double>( 1, 0 ) = tfRt.getOrigin()[1];
		  cvt_flea.at<double>( 2, 0 ) = tfRt.getOrigin()[2];
		  std::cout << tfRt.getOrigin()[0]<< " "<< tfRt.getOrigin()[1] << " "<< tfRt.getOrigin()[2]<<std::endl;

		  cv::Mat cvR( 3, 3, CV_64FC1 ); // rotation matrix
		  tf::Matrix3x3 tfR( tfRt.getRotation() );

		  for( int r=0; r<3; r++ ) {
		   for( int c=0; c<3; c++ ) {
		          cvR_flea.at<double>( r, c ) = tfR[r][c];
		   }
		  }

		  if(freeze_path){
			stduv = stduv_old;
		  }else if(stdxyz.size() != 0){
			cv::projectPoints(stdxyz , cvR_flea, cvt_flea, K_flea, D_flea, stduv );
			stduv_old.clear();
			stduv_old = stduv;
		  }
		  if(stdxyz_gui.size() != 0){
			cv::projectPoints(stdxyz_gui , cvR_flea, cvt_flea, K_flea, D_flea, stduv_gui );
		  }

		  for (int i = 0; i < stduv.size(); i++){		  
			if (show_markers){
				circle(img, Point(stduv[i].x, stduv[i].y), 2, Scalar(0,0,255), 2, LINE_AA);
				int thickness = 1;
				int lineType = LINE_8;
				Point start;
				start.x = stduv[i].x;
				start.y = stduv[i].y;
				Point end;
				end.x = stduv[(i+1)%stduv.size()].x;
				end.y = stduv[(i+1)%stduv.size()].y;
				if(marker_cog_pcl.points[i].intensity == 1){
					line( img,
					    start,
					    end,
					    Scalar( 255, 0, 0 ),
					    thickness,
					    lineType );
				}else{
					line( img,
					    start,
					    end,
					    Scalar( 0, 255, 0 ),
					    thickness,
					    lineType );
				}
			}
		  }
		  int clr = 255;
		  for (int i = 0; i < stduv_gui.size(); i++){
			if (i == 1)
				clr = 0;
			circle(img, Point(stduv_gui[i].x, stduv_gui[i].y), 4, Scalar(clr,255,clr), 4, LINE_AA);
		  }
	   	  Mat img_crop = img(roi);
		  std::string control_text;
   		  control_text = "Suggested Mode:";
		  putText(img_crop, control_text, Point(50, 50), CV_FONT_HERSHEY_DUPLEX, 1.5, Scalar(255, 255, 255), 2);	
   		  control_text = "- Auto";
		  putText(img_crop, control_text, Point(500, 50), CV_FONT_HERSHEY_DUPLEX, 1.5, Scalar(0, 255, 0), 2);	
   		  control_text = "- Teleop";
		  putText(img_crop, control_text, Point(750, 50), CV_FONT_HERSHEY_DUPLEX, 1.5, Scalar(255, 0, 0), 2);	
		  if(tele_op_mode){
			control_text = "Current Mode: Teleop";
			clr = 255;
		  }
		  else{
			control_text = "Current Mode: Autonomous";
			clr = 0;
		  }
		  putText(img_crop, control_text, Point(50, 100), CV_FONT_HERSHEY_DUPLEX, 1.5, Scalar(clr, 255-clr, 0), 2);	

		  Point arrow_start;
		  Point arrow_end;
		  arrow_start.x = 150;
		  arrow_start.y = 220;
		  int arrow_len = 100;
		  if(!tele_op_mode){
			  if(fabs(teleop_cmd.linear.x) > 0.0){
				arrow_end.x = arrow_start.x + sgn(teleop_cmd.linear.x)*arrow_len;
				arrow_end.y = arrow_start.y;
				arrowedLine(img_crop, arrow_start, arrow_end, Scalar(255, 0, 0), 2, LINE_AA);
			  }
			  if(fabs(teleop_cmd.linear.y) > 0.0){
				arrow_end.x = arrow_start.x;
				arrow_end.y = arrow_start.y + sgn(teleop_cmd.linear.y)*arrow_len ;
				arrowedLine(img_crop, arrow_start, arrow_end, Scalar(0, 255, 0), 2, LINE_AA);
			  }
			  if(fabs(teleop_cmd.linear.z) > 0.0){
				arrow_end.x = arrow_start.x + sgn(teleop_cmd.linear.z)*arrow_len*0.707;
				arrow_end.y = arrow_start.y - sgn(teleop_cmd.linear.z)*arrow_len*0.707 ;
				arrowedLine(img_crop, arrow_start, arrow_end, Scalar(0, 0, 255), 2, LINE_AA);
			  }

		 }
		  cv_ptr->image = img_crop;
		
        	  overlay_pub.publish(cv_ptr->toImageMsg());
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
	
}

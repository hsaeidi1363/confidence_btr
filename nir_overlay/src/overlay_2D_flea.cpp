// this codes overlays the 2D position of the NIR markers on the RGB image of intel camera (need to run the nir_overlay.launch before this)
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


using namespace cv;
using namespace std;
using namespace cv_bridge;


bool freeze_path = false;

void get_freeze(const std_msgs::Bool & _data){
	freeze_path = _data.data;	
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
}

pcl::PointCloud<pcl::PointXYZI> marker_cog_pcl;
std::vector< cv::Point3f > stdxyz;


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


        int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);

	// subscribe to the rectified rgb input (needs the calibration file read in the launch file for the camera)
	ros::Subscriber cam_sub = nh_.subscribe("/flea/camera/image_color",1,get_img);
	ros::Subscriber caminfo_sub = nh_.subscribe("flea/camera/camera_info",1,get_caminfo);
	ros::Subscriber pcl_sub = nh_.subscribe("/nir_overlay_intel/cog",1,get_pcl);
	ros::Subscriber freeze_path_sub = nh_.subscribe("/freeze_path",1,get_freeze);	
	
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
	while(ros::ok()){
		if (initialized && caminfo_received && pcl_received){ 
	    	  std::vector< cv::Point2f > stduv;
		  if(freeze_path){
			stduv = stduv_old;
		  }else if(stdxyz.size() != 0){
			tf::StampedTransform tfRt;
      			listener.lookupTransform( "flea", "intel", ros::Time(0), tfRt );


      			Eigen::Affine3d eigRt;
    			tf::transformTFToEigen( tfRt, eigRt );

     
     			 cvt_intel.at<double>( 0, 0 ) = tfRt.getOrigin()[0];
     			 cvt_intel.at<double>( 1, 0 ) = tfRt.getOrigin()[1];
     			 cvt_intel.at<double>( 2, 0 ) = tfRt.getOrigin()[2];
     			 std::cout << tfRt.getOrigin()[0]<< " "<< tfRt.getOrigin()[1] << " "<< tfRt.getOrigin()[2]<<std::endl;

     			 cv::Mat cvR( 3, 3, CV_64FC1 ); // rotation matrix
     			 tf::Matrix3x3 tfR( tfRt.getRotation() );

     			for( int r=0; r<3; r++ ) {
     			   for( int c=0; c<3; c++ ) {
			          cvR_intel.at<double>( r, c ) = tfR[r][c];
        		    }
      			}
			cv::projectPoints(stdxyz , cvR_intel, cvt_intel, K_intel, D_intel, stduv );
			stduv_old.clear();
			stduv_old = stduv;
		  }
		  for (int i = 0; i < stduv.size(); i++){		  
			if (show_markers)		
				circle(img, Point(stduv[i].x, stduv[i].y), 1, Scalar(0,255,0), 1, LINE_AA);
		  }
	   	  Mat img_crop = img(roi);
		  cv_ptr->image = img_crop;
        	  overlay_pub.publish(cv_ptr->toImageMsg());
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
	
}

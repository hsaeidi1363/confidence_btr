/****************************************************************************
 *
 * nirtracker.cpp 
 * 04-05-2013 
 *
 * Description:
 *
 * Implements tracking node based on ViSP vpDot "blob" trackers. 
 * 
 * Tested on NIR images obtained from JAI and Basler cameras, processed by nirthreshold.cpp
 * 
 * 08-12-2013 rev:
 * Fixes publishing of cog, moments. Adds publishing of edges (not tested)
 *
 * Authors:
 * Azad Shademan
 * a.shademan@ieee.org
 *
 *****************************************************************************/

#include <opencv2/stitching.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <visp_bridge/image.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayX.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

class NIRBlobTracker{

private:

  image_transport::ImageTransport it;
  image_transport::Subscriber     sub_image;
  
  ros::Subscriber sub_intel_laser; //for chacking if the laser on the intel D415 is off or not. we can only rely on NIR camera when laser is off

  ros::Publisher                  pub_polygon;
  std::vector<vpDot2>             blobs;

public:

  bool                            display_init;
  bool 							  laser_off;
  double 						  t0;
  double 						  t1;
  vpDisplay*                      vp_display;
  vpImage<unsigned char>          vp_image;

  NIRBlobTracker( ros::NodeHandle& nh ) : 
    it( nh ) {
    
    sub_image=it.subscribe( "/pylon_camera_node/image_raw", 1, &NIRBlobTracker::CallbackImage, this );
    sub_intel_laser = nh.subscribe("d415_laser_off", 1 , &NIRBlobTracker::CallbackLaser, this);

    pub_polygon=nh.advertise<geometry_msgs::PolygonStamped>( "/markers/cog", 100 );

    vp_display = new vpDisplayX;
    display_init = false;

  } 
  
  ~NIRBlobTracker(){ vp_display->closeDisplay(); }

   void CallbackLaser(const std_msgs::BoolConstPtr& msg){
    laser_off = msg->data;
    t0 = ros::Time::now().toSec();
   }
   void CallbackImage( const sensor_msgs::ImageConstPtr& msg ){
	t1 = ros::Time::now().toSec() -t0;
	if(laser_off && (t1 > 0.22)){
//		std::cout << " bagh bahg"<<std::endl;
		vp_image = visp_bridge::toVispImage( *msg );
		cv_bridge::CvImagePtr cvb_image = cv_bridge::toCvCopy( msg, msg->encoding );
		
		// The next two blocks are lame. They really should be in their own
		// callback
		if( !display_init ){
		  vp_image.init( msg->height, msg->width, 255 );
		  vp_display->init( vp_image );
		  vpDisplay::display( vp_image );
		  vp_display->setTitle( "Trackers" );
		  display_init = true;
		vpTime::wait(1);
		}

		// track the blobs

		for( std::size_t i=0; i<blobs.size(); i++ )
		  { blobs[i].setGraphics( true ); }
		
		for( std::size_t i=0; i<blobs.size(); i++){
		  try{ blobs[i].track( vp_image ); }
		  catch (...) 
		{ /*std::cout<<"blobs["<<i<<"].tracking failed."<<std::endl;*/ }
		}

		// *** display markers on the image *** //
		geometry_msgs::PolygonStamped polygon;
		polygon.header.stamp = ros::Time::now();
	   
		vpDisplay::display( vp_image );
		for( std::size_t i=0; i<blobs.size(); i++){
		  vpImagePoint cog = blobs[i].getCog();

		  geometry_msgs::Point32 p;
		  p.x = cog.get_u();
		  p.y = cog.get_v();
		  p.z = 0.0; 
		  polygon.polygon.points.push_back( p );

		  vpDisplay::displayCross( vp_image, cog, 10, vpColor::red );
		  std::list<vpImagePoint>::const_iterator it;
		  std::list<vpImagePoint> edges;
		  blobs[i].getEdges( edges ); 
		  for( it = edges.begin(); it != edges.end(); ++it) { 
		    vpDisplay::displayPoint( vp_image, *it, vpColor::red ); 
		  }    

		}
		
		vpDisplay::flush( vp_image );

		vpImagePoint pi;
		vpMouseButton::vpMouseButtonType bt;
		if( vpDisplay::getClick( vp_image,  pi, bt, false ) ){
		  vpDot2 blob;
		  blob.initTracking( vp_image, pi );
		  blobs.push_back( blob );
		}
		
		// publish the polygons
		pub_polygon.publish( polygon );
	}
    
  }
  
};

/*  
 * =============================
 */
int main( int argc, char *argv[]){
  
  ros::init( argc, argv, "visp");
  ros::NodeHandle nh;

  NIRBlobTracker tracker( nh );
  ros::spin();
  
  return 0;
}

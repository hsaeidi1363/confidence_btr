#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PolygonStamped.h> // look it up
#include <geometry_msgs/Polygon.h> // look it up
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/UInt8.h>

class NIROverlay {

  private:

    typedef message_filters::Subscriber<sensor_msgs::CameraInfo>     SubCInfo;
    typedef message_filters::Subscriber<geometry_msgs::PolygonStamped> SubCOG;
    typedef image_transport::SubscriberFilter                        SubImage;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2>    SubPCL;

    typedef 
      message_filters::sync_policies::ApproximateTime
      < sensor_msgs::CameraInfo, geometry_msgs::PolygonStamped, sensor_msgs::PointCloud2 > 
      //< sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::PointCloud2 > 
      Policy;

    typedef message_filters::Synchronizer< NIROverlay::Policy > Sync;

    // node handle
    ros::NodeHandle nh;              // node handle
    ros::NodeHandle nhp;             // private node handle

    // image transport
    image_transport::ImageTransport it; // to transport the NIR image

    NIROverlay::SubCInfo sub_cinfo;  // subscribe to NIR camera parameters
    NIROverlay::SubCOG   sub_cog;  // subscribe to NIR image
    //NIROverlay::SubImage sub_image;  // subscribe to NIR image
    NIROverlay::SubPCL   sub_pcl;    // subscribe to PCL

    NIROverlay::Sync sync;           // synchronize all the above

    ros::Publisher pub_cog;          // publish the resulting camera
    ros::Publisher pub_3Dpolygon_cog;
    ros::Publisher pub_dbg;   
    ros::Publisher pub_kuka_dbg;
    size_t seq;

    tf::TransformListener listener;
    tf::TransformListener listener_dbg; // for checking the transformation in kuka frame

    ros::Subscriber sub_pause_tracker; //AS
    std_msgs::UInt8 pause_tracker; //AS


  protected:

    // main callback function
    void Callback( const sensor_msgs::CameraInfo& msginfo,
        const geometry_msgs::PolygonStamped& cog,
        //const sensor_msgs::Image& msgimg,
        const sensor_msgs::PointCloud2& msgpcl );

    // pause/continue callback function //AS
    void pause_tracker_callback( const std_msgs::UInt8& msg); //AS


  public:

    NIROverlay( ros::NodeHandle nh, ros::NodeHandle nhp );

};

NIROverlay::NIROverlay( ros::NodeHandle nh, ros::NodeHandle nhp ) :
  nh( nh ),
  nhp( nhp ),
  it( nh ),
  sub_cinfo( nh, "/pylon_camera_node/camera_info", 1 ),
  sub_cog( nh, "/markers/cog", 1 ),
  //sub_image( it, "/basler/image_rect", 1 ),
  sub_pcl( nh, "/d415/filtered_points", 1 ),
  sync( NIROverlay::Policy(500), sub_cinfo, sub_cog, sub_pcl ),
  //sync( NIROverlay::Policy(100), sub_cinfo, sub_image, sub_pcl ),
  seq( 0 ) {

    sync.registerCallback( &NIROverlay::Callback, this );

    pub_cog=nhp.advertise< pcl::PointCloud<pcl::PointXYZI> >("cog", 1);
    pub_3Dpolygon_cog= nhp.advertise<geometry_msgs::Polygon>( "polygon3D_cog", 100 );
    pub_dbg=nhp.advertise< pcl::PointCloud<pcl::PointXYZI> >("overlayDBG", 1);
    pub_kuka_dbg=nhp.advertise< geometry_msgs::Polygon>("kuka_polygon", 1);
    std::cout << sub_cinfo.getTopic() << " " 
      << sub_cog.getTopic() << " " 
      << sub_pcl.getTopic() << std::endl;

    sub_pause_tracker = nhp.subscribe("/tracking/pause", 10, &NIROverlay::pause_tracker_callback, this); //AS


  }

void NIROverlay::Callback( const sensor_msgs::CameraInfo& msginfo,

			   const geometry_msgs::PolygonStamped& cog,
			   //const sensor_msgs::Image& msgimg,
			   const sensor_msgs::PointCloud2& msgpcl ) {

  std::cout << cog << std::endl;
  // what was about that pause tracker?

    try {  // get the pos/ori of raytrix wrt basler
      tf::StampedTransform tfRt;
      listener.lookupTransform( "basler", "camera_depth_optical_frame", ros::Time(0), tfRt );

      tf::StampedTransform tfRt_dbg; // for checking the transformations in robot frame
      listener_dbg.lookupTransform( "kuka", "intel", ros::Time(0), tfRt_dbg );
      geometry_msgs::Polygon kuka_polygon;
      geometry_msgs::Polygon polygon3D;

      Eigen::Affine3d eigRt;
      tf::transformTFToEigen( tfRt, eigRt );

      cv::Mat cvt( 3, 1, CV_64FC1 ); // translation
      cvt.at<double>( 0, 0 ) = tfRt.getOrigin()[0];
      cvt.at<double>( 1, 0 ) = tfRt.getOrigin()[1];
      cvt.at<double>( 2, 0 ) = tfRt.getOrigin()[2];
      std::cout << tfRt.getOrigin()[0]<< " "<< tfRt.getOrigin()[1] << " "<< tfRt.getOrigin()[2]<<std::endl;

      cv::Mat cvR( 3, 3, CV_64FC1 ); // rotation matrix
      tf::Matrix3x3 tfR( tfRt.getRotation() );

      for( int r=0; r<3; r++ ) {
        for( int c=0; c<3; c++ ) {
          cvR.at<double>( r, c ) = tfR[r][c];
        }
      }

      cv::Mat K = cv::Mat::zeros( 3, 3, CV_64FC1 );       // camera matrix
      K.at<double>( 0, 0 ) = msginfo.K[0];
      K.at<double>( 0, 2 ) = msginfo.K[2];
      K.at<double>( 1, 1 ) = msginfo.K[4];
      K.at<double>( 1, 2 ) = msginfo.K[5];
      K.at<double>( 2, 2 ) = 1.0;
	  
      cv::Mat D = cv::Mat::zeros( 1, 5, CV_64FC1 );       // camera distortion
      for( size_t i=0; i<5; i++ ) { D.at<double>( i ) = msginfo.D[i]; }

      // xyz-i pcl
      pcl::PointCloud<pcl::PointXYZI> pclxyzi;
	  pcl::PointCloud<pcl::PointXYZI> pcldbg;
      // convert ros->pcl
      pcl::fromROSMsg( msgpcl, pclxyzi );
	  pcl::fromROSMsg( msgpcl, pcldbg );
      pcl::PointCloud<pcl::PointXYZI>::iterator pi=pclxyzi.begin();

      // 3D coordinates 
      pcl::PointCloud<pcl::PointXYZI> pclcog;
      //pcl::PointCloud<pcl::PointXYZRGB> pclxyzrgb(pclxyzi.width,pclxyzi.height);
      //pcl::PointCloud<pcl::PointXYZRGB>::iterator prgb=pclxyzrgb.begin();

      std::vector< cv::Point2f > stduv;
      std::vector< cv::Point3f > stdxyz;

      //for( ; pi!=pclxyzi.end(); pi++, prgb++ ) {
	  for( ; pi!=pclxyzi.end(); pi++ ) {

        /*uint32_t rgb = ((uint32_t)(pi->intensity)<<16 | 
            (uint32_t)(pi->intensity)<<8  |
            (uint32_t)(pi->intensity));
        prgb->rgb = *reinterpret_cast<float*>(&rgb);
        prgb->x = pi->x;
        prgb->y = pi->y;
        prgb->z = pi->z;
		*/
        if( 0 < fabs( pi->x ) && 0 < fabs( pi->y ) && 0 < fabs( pi->z ) )
        { stdxyz.push_back( cv::Point3f ( pi->x, pi->y, pi->z ) ); }

      }

      cv::projectPoints( stdxyz, cvR, cvt, K, D, stduv );
      geometry_msgs::Point32 polygon3D_pt;
      std::vector< cv::Mat > cvcog( cog.polygon.points.size() );
      for( size_t j=0; j<cog.polygon.points.size(); j++ ){
        cv::Point2f cogj( cog.polygon.points[j].x, cog.polygon.points[j].y );
	//polygon3D_pt.x =cog.polygon.points[j].x;
	//polygon3D_pt.y =cog.polygon.points[j].y;
	//polygon3D.points.push_back(polygon3D_pt);
        cvcog[j] = cv::Mat( cogj );
      }

       for( size_t j=0; j< cog.polygon.points.size(); j++ ){
        // if the 3D point projection is within the NIR image, then overlay 
        // the NIR intensity on the point cloud +30 on the red channel   
        float min_dist = 1000000;
	      int min_ind = 0;
         
        for( size_t i=0; i< stduv.size(); i++ ){
            cv::Mat stduvi( stduv[i] );  //stduv is projected 2d points from 3D pointCloud
          //            user_clicked point,        point cloud projected   
        double dist =  cv::norm( cvcog[j], stduvi );
          
        if (dist < min_dist){
			       min_dist = dist;
			       min_ind = i;
             }
        } 
                    
         pcl::PointXYZI xyzi;
         xyzi.x = stdxyz[min_ind].x;
         xyzi.y = stdxyz[min_ind].y;
         xyzi.z = stdxyz[min_ind].z;
         xyzi.intensity = j;
         pclcog.push_back( xyzi );
         polygon3D_pt.x = xyzi.x;
	polygon3D_pt.y = xyzi.y;
	polygon3D_pt.z = xyzi.z;
	polygon3D.points.push_back(polygon3D_pt);

	//some debugging for the kuka frame coordinates of the COG
	tf::Vector3 pos_in_kuka;
	pos_in_kuka = tfRt_dbg(tf::Vector3(xyzi.x,xyzi.y,xyzi.z));
	geometry_msgs::Point32 kuka_pt;
	kuka_pt.x = pos_in_kuka.x();
	kuka_pt.y = pos_in_kuka.y();
	kuka_pt.z = pos_in_kuka.z();
	kuka_polygon.points.push_back(kuka_pt);
      }

      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.seq = seq++;
      header.frame_id = std::string( "/camera_depth_optical_frame" );
		
      pclcog.header = pcl_conversions::toPCL( header );
      pcldbg.header = pcl_conversions::toPCL( header );
      pub_cog.publish( pclcog );
      pub_dbg.publish(pcldbg);
      pub_3Dpolygon_cog.publish(polygon3D);
      pub_kuka_dbg.publish(kuka_polygon);

    }
    catch(...) { 
      std::cout << "no frame" << std::endl; 
    }
  
  
}

  // ---------------------------------------------------------------
  // Pause/continue tracker callback
  // ---------------------------------------------------------------
  void NIROverlay::pause_tracker_callback( const std_msgs::UInt8& msg) {
        pause_tracker = msg;
  }


int main( int argc, char** argv ){

  ros::init( argc, argv, "NIROverlay" );

  ros::NodeHandle nh, nhp("~");
  NIROverlay overlay( nh, nhp );

  ros::spin();

  return 0;

}


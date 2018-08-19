#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <std_msgs/String.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/photo/photo.hpp>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
tf::TransformListener *tfListener; 

bool BLUEBERRY = false;
std::string RGB_TOPIC = "/camera/rgb/image_raw";
cv::Mat rgb_im;
cv::Mat depth_im;
cv::Mat canny_im;
cv::Mat depth_im_mod;

cv_bridge::CvImagePtr cv_ptr;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy; //Changed
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
message_filters::Synchronizer<MySyncPolicy> *point_sync;

PointCloud cloud_filtered;


void all_cb ( const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& rgb_msg )
{
  try
  {
    rgb_im = cv_bridge::toCvCopy ( rgb_msg, enc::BGR8 ) -> image;
    depth_im = cv_bridge::toCvCopy ( depth_msg, enc::TYPE_16UC1 ) -> image;
    pcl::fromROSMsg(*cloud, cloud_filtered);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}

cv::Mat filter_depth ( int max_dist )
{
  cv::Mat depth_im_mod = depth_im.clone();
  double depth_mm, depth_m;

  for ( unsigned int i = 0; i < depth_im.rows; i++ )
  {
    for ( unsigned int j = 0; j < depth_im.cols; j++ )
    {
      depth_mm = depth_im.at<uint16_t>(i,j);
      depth_m = (float)depth_mm/1000;

      if ( depth_m > max_dist )
        depth_im_mod.at<uint16_t>(i,j) = 0;
    }
  }

  return depth_im_mod;
}

Point3f get_kinect_coordinates ( int i, int j )
{
  Point3f coordinate;
  coordinate.x = cloud_filtered.at(i, j).x;
  coordinate.y = cloud_filtered.at(i, j).y;
  coordinate.z = cloud_filtered.at(i, j).z;

  return coordinate;
}


geometry_msgs::PoseStamped createPose(std::string frame, double x, double y, double z, double ax, double ay, double az) 
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = ax;
  pose.pose.orientation.y = ay;
  pose.pose.orientation.z = az;
  pose.pose.orientation.w = 1.0;
  pose.header.stamp = ros::Time::now();

  return pose;
}

geometry_msgs::PoseStamped transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom) 
{
  geometry_msgs::PoseStamped poseTo;
  try 
  {
    ros::Time current_time = ros::Time::now();
    poseFrom.header.stamp = current_time;
    poseTo.header.stamp = current_time;
    tfListener->waitForTransform(frameTo, poseFrom.header.frame_id, current_time, ros::Duration(1.0));
    tfListener->transformPose(frameTo, poseFrom, poseTo);
  } 
  catch (tf::TransformException ex)
  {
  	std::cout << ex.what() << std::endl;
  }
  return poseTo;
}


Point3f frame_transform ( Point3f kinect_frame_coordinates, std::string destination_frame )
{
  Point3f new_frame_coordinates;

	std::string frame_name = "camera_depth_optical_frame";

  if ( BLUEBERRY )
  {
    geometry_msgs::PoseStamped kinect_pose = createPose ( frame_name, kinect_frame_coordinates.x, kinect_frame_coordinates.y, kinect_frame_coordinates.z, 0, 0, 0);
    geometry_msgs::PoseStamped new_pose = transformPose ( destination_frame, kinect_pose );
    new_frame_coordinates.x = new_pose.pose.position.x;
    new_frame_coordinates.y = new_pose.pose.position.y;
    new_frame_coordinates.z = new_pose.pose.position.z; 
  }

  else if ( destination_frame == frame_name )
  {
    geometry_msgs::PoseStamped kinect_pose = createPose ( frame_name, kinect_frame_coordinates.x, kinect_frame_coordinates.y, kinect_frame_coordinates.z, 0, 0, 0);
    new_frame_coordinates.x = kinect_pose.pose.position.x;
    new_frame_coordinates.y = kinect_pose.pose.position.y;
    new_frame_coordinates.z = kinect_pose.pose.position.z; 
  }

  else
    new_frame_coordinates = kinect_frame_coordinates;

  return new_frame_coordinates;


}

void draw_coordinate ( int i, int j, Scalar color )
{
  //Scalar color = (0, 255, 0);
  cv::circle(rgb_im, cv::Point(i,j), 3, color, 5, 8, 0 );

  Point3f kinect_coordinates = get_kinect_coordinates ( i,j );
  Point3f camera_link_coordinates = frame_transform ( kinect_coordinates, "camera_link" );
  Point3f map_coordinates = frame_transform ( kinect_coordinates, "map" );
  Point3f base_link_coordinates = frame_transform ( kinect_coordinates, "base_link" );

  std::string kinect_coordinates_text = cv::format("Kinect:     (%.1f, %.1f, %.1f)", kinect_coordinates.x, kinect_coordinates.y, kinect_coordinates.z );
  std::string camera_link_coordinates_text = cv::format("cam_link: (%.1f, %.1f, %.1f)", camera_link_coordinates.x, camera_link_coordinates.y, camera_link_coordinates.z );
  std::string map_coordinates_text = cv::format("Map:        (%.1f, %.1f, %.1f)", map_coordinates.x, map_coordinates.y, map_coordinates.z );
  std::string base_link_coordinates_text = cv::format("base_link: (%.1f, %.1f, %.1f)", base_link_coordinates.x, base_link_coordinates.y, base_link_coordinates.z );

  int pos_x = i - 40;
  int pos_y = j - 75;

  cv::putText(rgb_im, kinect_coordinates_text, cv::Point(pos_x, pos_y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 2.0);
  
  pos_y += 20;  
  cv::putText(rgb_im, camera_link_coordinates_text, cv::Point(pos_x, pos_y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 2.0);

  pos_y += 20;  
  cv::putText(rgb_im, base_link_coordinates_text, cv::Point(pos_x, pos_y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 2.0);

  pos_y += 20;
  cv::putText(rgb_im, map_coordinates_text, cv::Point(pos_x, pos_y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 2.0);
}

void draw_image ( void )
{
  cv::imshow ( "Image", rgb_im );
  cv::waitKey(1);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DepthFilter");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
  	tf::TransformListener temp_tfListener; 
 		tfListener = &temp_tfListener; 

    message_filters::Subscriber<sensor_msgs::PointCloud2> pts_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;

    depth_sub.subscribe(n, "/camera/depth/image_raw", 1);
    pts_sub.subscribe(n, "/camera/depth/points", 1);
    rgb_sub.subscribe(n, RGB_TOPIC, 1);

    point_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy( 100 ), depth_sub, pts_sub, rgb_sub);
    point_sync->registerCallback( boost::bind( all_cb, _1, _2, _3));

 		sleep(2);

	 	while ( ros::ok() )
 		{
	 		ros::Time current_time = ros::Time::now();
	 		double duration;

	 		ros::spinOnce();
	 		
	    int I = 100;
	    int J = 380;
	    int Ic = 400;
	    int Jc = 300;
	    Scalar Bcolor = Scalar(255, 0, 0);
	    Scalar Gcolor = Scalar(0, 255, 0);
	    Scalar Rcolor = Scalar(0, 0, 255);

      if ( !depth_im.empty() )
      {
        draw_coordinate ( I, J, Bcolor );
        draw_coordinate ( Ic, Jc, Rcolor );
        draw_image ();
      }

      duration = ( ros::Time::now().toSec() - current_time.toSec() );
      std::cout << "Run time: " << duration << std::endl;      
    }

    return 0;
}



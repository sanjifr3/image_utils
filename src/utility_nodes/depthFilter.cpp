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

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

cv::Mat rgb_im;
cv::Mat depth_im;
cv::Mat canny_im;
cv::Mat depth_im_mod;
cv::Mat detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 30;
int ratio = 2;
int kernel_size = 3;
std::string window_name = "Edge Map";

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



/*
  for ( unsigned int i = 0; i < depth_im.rows; i++ )
  {
    for ( unsigned int j = 0; j < depth_im.cols; j++ )
    {
      //std::cout << "(" << i << ", " << j << ")" << std::endl;
      depth_im.at<int>(i,j) = 255;
      //if (cloud_filtered.at(i,j).z > 3.0)
      //  depth_im.at<int>(i,j) == 255;
    }
  }
*/





/*
  Scalar color = (0, 255, 0);
  circle(image_rgb, cv::Point(I,J), 3, color, 5, 8, 0);

  Scalar color2 = (0, 0, 255);
  circle(image_rgb, cv::Point(320,240), 3, color2, 5, 8, 0);

  double x = cloud_filtered.at(I,J).x;
  double y = cloud_filtered.at(I,J).y;
  double z = cloud_filtered.at(I,J).z;

  std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
*/

}

void depth_cb (const sensor_msgs::ImageConstPtr& msg)
{
	try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);
    depth_im = cv_ptr->image;
    //now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
  }

	catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //depth_im = cv_ptr->image;
	//uint16_t depth_mm = cv_ptr->image.at<uint16_t>(cv::Point(I,J));
  

  //float a = 0.00173667;
  //float z = (float)depth_mm/1000;
  //float x = (I-320)*a*z;
  //float y = (J-240)*a*z;

  //std::cout << "Math one: " << "x: " << x << ", y: " << y << ", z: " << z << std::endl;

}

void CannyThreshold ( int, void* )
{
  // Reduce noise with a kernal 3x3
  cv::blur ( depth_im_mod, detected_edges, Size(3,3) );

  // Canny detector
  cv::Canny ( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  // Using Canny's output as a mask, we display our result
  canny_im = Scalar::all(0);

  depth_im_mod.copyTo ( canny_im, detected_edges );
  cv::imshow ( window_name, canny_im );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DepthFilter");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
/*
    message_filters::Subscriber<sensor_msgs::PointCloud2> pts_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;

    depth_sub.subscribe(n, "/camera/depth/image_raw", 1);
    pts_sub.subscribe(n, "/camera/depth/points", 1);
    rgb_sub.subscribe(n, "/camera/rgb/image_raw", 1);

    point_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy( 100 ), depth_sub, pts_sub, rgb_sub);
    point_sync->registerCallback( boost::bind( all_cb, _1, _2, _3));
*/
    ros::Subscriber depth_sub = n.subscribe( "/camera/depth/image_raw", 1, depth_cb);

    std::clock_t start_time = std::clock();
    double duration;
    ros::Rate r(1000);



    while ( ros::ok() )
    {
      start_time = std::clock();
      ros::spinOnce();
      
      //r.sleep();

      if ( !depth_im.empty() )
      {        
        uint16_t depth_mm;// = cv_ptr->image.at<uint16_t>(cv::Point(I,J));
        float depth_m;

        depth_im_mod = depth_im.clone();
        
        for ( unsigned int i = 0; i < depth_im.rows; i++)
        {
          for ( unsigned int j = 0; j < depth_im.cols; j++)
          {
            depth_mm = depth_im.at<uint16_t>(i,j);
            depth_m = (float)depth_mm/1000;
            if ( depth_m > 2.5 )
              depth_im_mod.at<uint16_t>(i,j) = 0;
          }
        }

        double min, max;
        minMaxIdx(depth_im_mod, &min, &max); // finds min and max values in image_depth
        convertScaleAbs(depth_im_mod, depth_im_mod, 255/max); // Change scale
        depth_im_mod.convertTo(depth_im_mod, CV_8UC1, 255/(max-min), -min*255/(max-min));

        //cv::fastNlMeansDenoising ( depth_im_mod, depth_im_mod, 3, 5, 11 );
        //cv::GaussianBlur ( depth_im_mod, depth_im_mod, Size(5,5), 0.0, 0.0, BORDER_DEFAULT );
        //cv::medianBlur ( depth_im_mod, depth_im_mod, 5 );
        //cv::GaussianBlur ( depth_im_mod, depth_im_mod, Size(5,5), 0.0, 0.0, BORDER_DEFAULT );
        //cv::blur ( depth_im_mod, depth_im_mod, Size(3,3) );

        canny_im.create ( depth_im_mod.rows, depth_im_mod.cols, depth_im_mod.depth() );

        namedWindow( window_name, CV_WINDOW_AUTOSIZE );

        createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

        CannyThreshold(0, 0);

        waitKey(1);
      }   
      
      duration = ( std::clock() - start_time ) / (double) CLOCKS_PER_SEC;
      std::cout << "Run Time: " << duration << std::endl;
    }

    return 0;
}
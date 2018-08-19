// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <stereo_msgs/DisparityImage.h>
#include <ros/package.h>
// STANDARD
//#include <omp.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
//#include <sstream>
// BOOST
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
// OPENCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/photo/photo.hpp>
//#include "cv.h"
//#include "cvaux.h"
#include <image_transport/image_transport.h>
// OTHER
#include <stdio.h>
#include <stdlib.h>
//#include <termios.h>
//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>

//#ifndef BOOL
  //#define BOOL BOOL
//#endif

using namespace std;
using namespace cv;

// To create list of training files:
// cd ~/silhouette_detection/src/social_robot/bagfiles
// find ~/silhouette_detection/src/social_robot/bagfiles/plain -type f -maxdepth 1 > plain.txt
// find ~/silhouette_detection/src/social_robot/bagfiles/crowded -type f -maxdepth 1 > crowded.txt

static void read_file ( const string &filename, vector<string> &bagfile_path )
{
  string line;
  ifstream myfile( filename.c_str() );
  if( myfile.is_open() )
  {
    while( getline( myfile, line ) )
      bagfile_path.push_back( line.c_str() );

    myfile.close();
  }
}

void disparitycb (const stereo_msgs::DisparityImagePtr disparity)
{

    cv::Mat_<float> disparityMat(disparity->image.height, disparity->image.width,
        reinterpret_cast<float*>(&(disparity->image.data[0])));

    cv::Mat_<uint8_t> eightBitDisparityMat = disparityMat * (255/disparity->max_disparity);

    sensor_msgs::Image eightBitDisparity;

    uint32_t imageSize = disparity->image.height * disparity->image.width;

    eightBitDisparity.data.resize(imageSize);
    memcpy(&eightBitDisparity.data[0], &(eightBitDisparityMat.at<uint8_t>(0,0)), imageSize);

    // Populate the rest of the sensor_msgs::Image fields
}

int main ( int argc, char **argv )
{
  std::cout << "Place optional .bag/.txt file in social_robot/bagfiles" << std::endl;
  string package_path = ros::package::getPath ( "social_robot" );
  string bagfiles_path;
  bagfiles_path.append ( package_path );
  bagfiles_path.append ( "/bagfiles/" );

  string file_name;
  file_name.append ( bagfiles_path );

  string plain_list;
  plain_list.append ( bagfiles_path );
  plain_list.append ( "plain.txt" );

  string crowded_list;
  crowded_list.append ( bagfiles_path );
  crowded_list.append ( "crowded.txt" );

  string old_bagfile_list;
  old_bagfile_list.append ( bagfiles_path );
  old_bagfile_list.append ( "old_bagfile_list.txt" );

  string ext_list;
  ext_list.append ( bagfiles_path );

  if ( argc == 2 )
  {
    std::string str ( argv[1] );
    std::size_t found;
 
    found = str.find("bag");
    if ( found != std::string::npos )
    {
      file_name.append ( argv[1] );
      stringstream ss;
      //ss << "rosbag play -l " << file_name;
      ss << "rosbag play -l " << file_name;
      system ( ss.str().c_str() );
      return 0;
    }

    found = str.find(".txt");
    if ( found != std::string::npos )
      ext_list.append ( argv[1] );

    found = str.find("none");
    if ( found != std::string::npos )
      argc = 1;
  }

  if ( argc > 2 )
  {
    std::cout << "Too much arguments!!" << std::endl;
    return 0;
  }

  ros::init( argc, argv, "bag_reader" );
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_rgb = it.advertise("/camera/rgb/image_raw", 1);
  image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_raw", 1);
  image_transport::Publisher pub_disp = it.advertise("/camera/depth_registered/disparity", 1);


  vector<string> crowded_bagfiles;
  vector<string> plain_bagfiles;
  vector<string> ext_bagfiles;
  vector<string> bagfiles;
  vector<string> old_bagfiles;

  if ( argc == 2 )
  {
    read_file ( ext_list, ext_bagfiles );
    sort( ext_bagfiles.begin(), ext_bagfiles.end() );
    bagfiles.insert( bagfiles.end(), ext_bagfiles.begin(), ext_bagfiles.end() );
  }

  else
  {
    read_file ( crowded_list, crowded_bagfiles );
    read_file ( plain_list, plain_bagfiles );
    read_file ( old_bagfile_list, old_bagfiles );
    
    sort ( crowded_bagfiles.begin(), crowded_bagfiles.end() );
    sort ( plain_bagfiles.begin(), plain_bagfiles.end() );

    bagfiles.insert(bagfiles.end(), plain_bagfiles.begin(), plain_bagfiles.end());
    bagfiles.insert(bagfiles.end(), crowded_bagfiles.begin(), crowded_bagfiles.end());
  }

  //while(true)
  //{ 
    for ( unsigned int i = 0; i < bagfiles.size(); i++ )
    {
      rosbag::Bag bag;
      std::vector<std::string> topics;

      bag.open( bagfiles[i].c_str(), rosbag::bagmode::Read);
      //bag.open("/home/sanjif-raj/silhouette_detection/src/social_robot/test2.bag", rosbag::bagmode::Read);
      
      topics.push_back(std::string("/camera/rgb/image_color"));
      //topics.push_back(std::string("/camera/rgb/image_raw"));
      topics.push_back(std::string("/camera/depth/image_raw"));
      //topics.push_back(std::string("/camera/depth_registered/disparity"));
      //topics.push_back(std::string("/camera/depth/disparity"));
      //topics.push_back(std::string("/camera/ir/image_rect_ir"));
      //topics.push_back(std::string("/clock"));

      //topics.push_back(std::string("/camera/rgb/image_raw"));
      //topics.push_back(std::string("/camera/depth/image_raw"));
      //topics.push_back(std::string("/camera/depth_registered/disparity"));
      //topics.push_back(std::string("/camera/depth/disparity"));

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      int count = 0;

      cv::Mat rgb_mat;
      cv::Mat depth_mat;
      cv::Mat disp_mat;

      foreach(rosbag::MessageInstance const m, view)
      {
        sensor_msgs::ImageConstPtr im = m.instantiate<sensor_msgs::Image>();
        stereo_msgs::DisparityImageConstPtr s_im = m.instantiate<stereo_msgs::DisparityImage>();
        
        if ( count == 0 )
        {
          if ( im != NULL )
          {
            depth_mat = cv_bridge::toCvCopy( im ) -> image;
            count++;
          }
        }

        else if ( count == 1 )
        {
          if ( im != NULL )
          {
            rgb_mat = cv_bridge::toCvCopy( im ) -> image;
            count++;
          }
        }

        if ( s_im != NULL )
        {
          disp_mat = cv_bridge::toCvCopy( s_im -> image ) -> image;
          disp_mat.convertTo ( disp_mat, CV_8UC1 );
        }
      }

      //Publishing
      sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_mat).toImageMsg();
      sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_mat).toImageMsg();
      sensor_msgs::ImagePtr msg_disp = cv_bridge::CvImage(std_msgs::Header(), "mono8", disp_mat).toImageMsg();

      cv::waitKey(1000);
      ros::Rate loop_rate(1000);
      loop_rate.sleep();
      pub_rgb.publish ( msg_rgb );
      pub_depth.publish ( msg_depth );
      pub_disp.publish ( msg_disp );

      bag.close();
    }
//}
  return 0;
}
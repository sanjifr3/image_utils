#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>

#include <social_robot/utils/kinect_proxy.h>
#include <social_robot/utils/CvUtils.h>
#include <social_robot/utils/RosUtils.h>
#include <social_robot/utils/string_utils.h>

using namespace std;
using namespace cv;

namespace enc = image_encodings;

VideoWriter rgbwriter;

CvUtils cv_utils;
bool isfirstframe = true;
string video_file_name;

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;

      if ( isfirstframe )
        {
          int width = image_rgb.cols;
          int height = image_rgb.rows;
          rgbwriter.open ( video_file_name, CV_FOURCC ( 'D', 'I', 'V', 'X' ), 30, Size ( width, height ) );
          if ( !rgbwriter.isOpened() )
            {
              cerr << "Could not open '" << "'" << endl;
            }
          isfirstframe = false;
        }

      if ( rgbwriter.isOpened() )
        {
          rgbwriter << image_rgb;
        }

    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "rgbwriter" );
  ros::NodeHandle nh;

  ROS_INFO("rgbWriter has started");

  string package_path = ros::package::getPath ( "social_robot" );
  video_file_name = package_path + "/results/";
  
  if ( argc > 1 )
  {
    video_file_name.append ( argv[1] );
    video_file_name.append ( ".avi" );
  }

  else
  	video_file_name = video_file_name + "default.avi";

  // subscribtions
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_raw", 1, rgb_cb );

  double duration;
  ros::Time start_time = ros::Time::now();

  ros::spin();

  return 0;
}

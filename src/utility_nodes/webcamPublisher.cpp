#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "webcam_publisher");
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/cam/rgb/image_raw", 1);
  VideoCapture cap(0);
  
  if( !cap.isOpened() )
  {
    std::cout << "Cannot open the webcam" << std::endl;
    return -1;
  }

  for(;;)
  {
    Mat frame;
    bool ReadSuccess = cap.read(frame);

    if(!ReadSuccess)
    {
      std::cout << "Cannot read a frame from video stream" << std::endl;
      break;
    }
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    ros::Rate loop_rate(5);
    pub.publish(msg);
  }

  return 0;
}
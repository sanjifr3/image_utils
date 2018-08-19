#include <image_utils/Utilities.h>
#include <image_utils/ImageGrabber.h>
#include <image_utils/ImageProcessor.h>
#include <image_utils/PCLUtilities.h>

bool use_topic = true;

int main(int argc, char** argv){
  ros::init(argc, argv, "utils_test_node");
  ros::NodeHandle nh;
  
  ImageGrabber ig(nh);
  ImageGrabber head_cam_topic(nh, "head_camera/rgb/image_raw");
  ImageGrabber head_cam(nh, 3);
  ImageProcessor ip;
  PCLUtilities pcl(nh);

  while(ros::ok()){
    ig.grabImage();
    
    if(!ig.rgb_im_.empty()){
      ip.convertTo(ig.rgb_im_);
      cv::imshow("ig",ig.rgb_im_);
    }

    if(use_topic){
      head_cam_topic.grabImage();
      if(!head_cam_topic.rgb_im_.empty())
        cv::imshow("hc_t",head_cam_topic.rgb_im_);      
    }

    else{
      head_cam.grabImage();
      if(!head_cam.rgb_im_.empty())
        cv::imshow("hc",head_cam.rgb_im_);
    }

    cv::waitKey(1);
  }
  
  return 0;
}
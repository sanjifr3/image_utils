#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include <image_utils/Utilities.h>
#include <image_utils/GeneralRQTConfig.h>
#include <opencv2/cudawarping.hpp>

// ROS Libraries
#include <sensor_msgs/PointCloud2.h>

// RQT Dynamic Reconfigure Libraries
#include <dynamic_reconfigure/server.h>
#include <image_utils/GeneralRQTConfig.h>

// Image Transport Libraries
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Message Filter Libraries
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class ImageGrabber{
  private:
    // Subscribers
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyDualSyncPolicy_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MyTripleSyncPolicy_;
    
  public:
    ImageGrabber(ros::NodeHandle nh);
    ImageGrabber(ros::NodeHandle nh, int device);
    ImageGrabber(ros::NodeHandle nh, int device, std::string name);
    ImageGrabber(ros::NodeHandle nh, std::string topic);
    ImageGrabber(ros::NodeHandle nh, std::string topic, bool set_name, std::string name);
    ImageGrabber(ros::NodeHandle nh, std::string t1, std::string t2);
    ImageGrabber(ros::NodeHandle nh, std::string t1, std::string t2, std::string t3);
    ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh);
    ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, int device);
    ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string topic);
    ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string t1, std::string t2);
    ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string t1, std::string t2, std::string t3);    
    ~ImageGrabber();
         
    cv::Mat grabImageCopy(bool get_new=false, bool block_cb=false);
    cv::cuda::GpuMat grabGPUImageCopy(bool get_new = false, bool block_cb=false);
    
    void grabImage(bool block_cb=false); 
    
    void show(bool depth=false, bool gpu=false, int wait=1);

    void mergeDepthFrames(int& indx);
    void colorBalance(double pct=10.0, bool cb_gpuMat=false);
    
    void createCVWindow(std::string window_name);
    void destroyCVWindow(std::string window_name);
    void destroyAllCVWindows();
    void openVideoWriter(cv::VideoWriter &writer, std::string file_name);

    void freeCb();
    void blockCb();
  
  private:
    void rescale();
  
    void imCb(const sensor_msgs::Image::ConstPtr& msg);
    void dualSubCb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg);
    void tripleSubCb (const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::PointCloud2ConstPtr& cloud);

    void loadROSParams(std::string ns=ros::this_node::getName());
    void rqtCb(image_utils::GeneralRQTConfig &config, uint32_t level);

  public:    
    cv::Mat rgb_im_, d_im_;
    PointCloud_ cloud_;
    cv::cuda::GpuMat rgb_gim_, d_gim_;

    std::vector<cv::Mat> depth_frames_;
    
    cv::Size rgb_im_size_ = cv::Size(1280,720); //cv::Size(640,480);
    cv::Size d_im_size_ = cv::Size(1280,720); //cv::Size(640,480);
    
  private:        
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv::VideoCapture cap_;
        
    dynamic_reconfigure::Server<image_utils::GeneralRQTConfig> server_;
    dynamic_reconfigure::Server<image_utils::GeneralRQTConfig>::CallbackType f_;

    std::string name_ = "Jetson";
    std::string device_ = "Default";
    std::string rgb_topic_ = "camera3D/color/image_rect_color"; //"camera/rgb/image_color";
    //std::string depth_topic_ = "camera3D/depth/image_rect_raw"; //"camera/depth/image_raw";
    std::string depth_topic_ = "camera3D/aligned_depth_to_color/image_raw"; //camera3D/depth/image_rect_raw
    std::string pts_topic_ = "camera3D/depth_registered/points"; //"camera/depth/points";
      
    image_sub_ rgb_sub_;
    image_sub_ depth_sub_;
    cloud_sub_ pts_sub_;
    image_transport::Subscriber im_sub_;
    
    image_transport::Publisher im_pub_;

    bool save_video_ = false;
    bool show_video_ = true;
    bool pub_video_ = true;
    
    message_filters::Synchronizer<MyDualSyncPolicy_> *dual_sync_;
    message_filters::Synchronizer<MyTripleSyncPolicy_> *triple_sync_;

    bool new_image_ = false;
    bool cb_blocked_ = false;
    bool color_balance_ = true; // Only for realsense
};

#endif //IMAGE_GRABBER_H 

#include <image_utils/ImageGrabber.h>

ImageGrabber::ImageGrabber(ros::NodeHandle nh) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  rgb_sub_.subscribe(nh_, rgb_topic_, 1);
  depth_sub_.subscribe(nh_, depth_topic_, 1);
  pts_sub_.subscribe(nh_, pts_topic_, 1);

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  triple_sync_ = new message_filters::Synchronizer<MyTripleSyncPolicy_>(MyTripleSyncPolicy_(100), rgb_sub_, depth_sub_, pts_sub_ );
  triple_sync_->registerCallback( boost::bind ( &ImageGrabber::tripleSubCb, this, _1, _2, _3 ));
  ROS_INFO("[ImageGrabber] Initialized with rgb/depth & pt_cloud subscribers");
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, int device) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  cap_.open(device);

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  if(cap_.isOpened())
    ROS_INFO("[ImageGrabber] Initialized with device %d",device);
  else
    ROS_INFO("[ImageGrabber] Failed with initialize with device %d", device);
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, int device, std::string name) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  device_ = name;

  cap_.open(device);

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  if(cap_.isOpened())
    ROS_INFO("[ImageGrabber] Initialized with device %d",device);
  else
    ROS_INFO("[ImageGrabber] Failed with initialize with device %d", device);
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, std::string topic) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  im_sub_ = it_.subscribe(topic, 1, &ImageGrabber::imCb, this);
  
  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with subscriber to %s", topic.c_str());  
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, std::string topic, bool set_name, std::string name) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  if(set_name) device_ = name;

  im_sub_ = it_.subscribe(topic, 1, &ImageGrabber::imCb, this);
  
  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with subscriber to %s", topic.c_str());  
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, std::string t1, std::string t2) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  rgb_sub_.subscribe(nh_, t1, 1);
  depth_sub_.subscribe(nh_, t2, 1);

  dual_sync_ = new message_filters::Synchronizer<MyDualSyncPolicy_>(MyDualSyncPolicy_(100), rgb_sub_, depth_sub_ );
  dual_sync_->registerCallback( boost::bind ( &ImageGrabber::dualSubCb, this, _1, _2 ));

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with rgb/depth subscribers");
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, std::string t1, std::string t2, std::string t3) : it_(nh) {
  this->nh_ = nh;
  loadROSParams();

  rgb_sub_.subscribe(nh_, t1, 1);
  depth_sub_.subscribe(nh_, t2, 1);
  pts_sub_.subscribe(nh_, t3, 1);

  triple_sync_ = new message_filters::Synchronizer<MyTripleSyncPolicy_>(MyTripleSyncPolicy_(100), rgb_sub_, depth_sub_, pts_sub_ );
  triple_sync_->registerCallback( boost::bind ( &ImageGrabber::tripleSubCb, this, _1, _2, _3 ));

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with rgb/depth & pt_cloud subscribers");
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh) : it_(nh), server_(rqt_nh){
  this->nh_ = nh;
  loadROSParams();

  f_ = boost::bind(&ImageGrabber::rqtCb, this, _1, _2);
  server_.setCallback(f_);

  rgb_sub_.subscribe(nh_, rgb_topic_, 1);
  depth_sub_.subscribe(nh_, depth_topic_, 1);
  pts_sub_.subscribe(nh_, pts_topic_, 1);

  ROS_INFO("[ImageGrabber] (RGB) Subscribed to %s topic", rgb_topic_.c_str());
  ROS_INFO("[ImageGrabber] (Depth) Subscribed to %s topic", depth_topic_.c_str());
  ROS_INFO("[ImageGrabber] (PCL) Subscribed to %s topic", pts_topic_.c_str());  

  triple_sync_ = new message_filters::Synchronizer<MyTripleSyncPolicy_>(MyTripleSyncPolicy_(100), rgb_sub_, depth_sub_, pts_sub_ );
  triple_sync_->registerCallback( boost::bind ( &ImageGrabber::tripleSubCb, this, _1, _2, _3 ));

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with rgb/depth & pt_cloud subscribers & rqt param server");
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, int device) : it_(nh), server_(rqt_nh){
  this->nh_ = nh;
  loadROSParams();

  f_ = boost::bind(&ImageGrabber::rqtCb, this, _1, _2);
  server_.setCallback(f_);

  cap_.open(device);

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  if(cap_.isOpened())
    ROS_INFO("[ImageGrabber] Initialized with device %d",device);
  else
    ROS_INFO("[ImageGrabber] Failed with initialize with device %d & rqt param server", device);
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string topic) : it_(nh), server_(rqt_nh){
  this->nh_ = nh;
  loadROSParams();

  f_ = boost::bind(&ImageGrabber::rqtCb, this, _1, _2);
  server_.setCallback(f_);

  im_sub_ = it_.subscribe(topic, 1, &ImageGrabber::imCb, this);

  std::vector<std::string> topic_split = utils::split(topic, '/');

  name_ = topic_split[0];

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with subscriber to %s & rqt param server", topic.c_str());  
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string t1, std::string t2) : it_(nh), server_(rqt_nh){
  this->nh_ = nh;
  loadROSParams();

  f_ = boost::bind(&ImageGrabber::rqtCb, this, _1, _2);
  server_.setCallback(f_);

  rgb_sub_.subscribe(nh_, t1, 1);
  depth_sub_.subscribe(nh_, t2, 1);

  dual_sync_ = new message_filters::Synchronizer<MyDualSyncPolicy_>(MyDualSyncPolicy_(100), rgb_sub_, depth_sub_ );
  dual_sync_->registerCallback( boost::bind ( &ImageGrabber::dualSubCb, this, _1, _2 ));

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with rgb/depth subscribers & rqt param server");
}

ImageGrabber::ImageGrabber(ros::NodeHandle nh, ros::NodeHandle rqt_nh, std::string t1, std::string t2, std::string t3) : it_(nh), server_(rqt_nh){
  this->nh_ = nh;
  loadROSParams();

  f_ = boost::bind(&ImageGrabber::rqtCb, this, _1, _2);
  server_.setCallback(f_);

  rgb_sub_.subscribe(nh_, t1, 1);
  depth_sub_.subscribe(nh_, t2, 1);
  pts_sub_.subscribe(nh_, t3, 1);

  triple_sync_ = new message_filters::Synchronizer<MyTripleSyncPolicy_>(MyTripleSyncPolicy_(100), rgb_sub_, depth_sub_, pts_sub_ );
  triple_sync_->registerCallback( boost::bind ( &ImageGrabber::tripleSubCb, this, _1, _2, _3 ));

  if(pub_video_)
    im_pub_ = it_.advertise(name_ + "/results/rgb",1,true);

  ROS_INFO("[ImageGrabber] Initialized with rgb/depth & pt_cloud subscribers & rqt param server");
}

ImageGrabber::~ImageGrabber(){
  if (cap_.isOpened()) cap_.release();
}

cv::Mat ImageGrabber::grabImageCopy(bool get_new, bool block_cb){
  if (cap_.isOpened()){
    cap_ >> rgb_im_;
    rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
    //rescale();
  }
  else if (get_new){
    ros::Rate r(100);
    while(!new_image_ && !cb_blocked_ && ros::ok()) ros::spinOnce(); r.sleep();
    new_image_ = false;
    if(block_cb) cb_blocked_ = true;
  }

  return rgb_im_;
}

cv::cuda::GpuMat ImageGrabber::grabGPUImageCopy(bool get_new, bool block_cb){
  if (cap_.isOpened()){
    cap_ >> rgb_im_;
    rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
    //rescale();
  }
  else if(get_new){
    ros::Rate r(100);
    while(!new_image_ && !cb_blocked_ && ros::ok()) ros::spinOnce(); r.sleep();
    new_image_ = false;
    if(block_cb) cb_blocked_ = true;
  }

  return rgb_gim_;
}

void ImageGrabber::grabImage(bool block_cb){
  if (cap_.isOpened()){
    cap_ >> rgb_im_;
    rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
    //rescale();
  }
  else{
    ros::Rate r(100);
    while(!new_image_ && !cb_blocked_ && ros::ok()) ros::spinOnce(); r.sleep();
    new_image_ = false;
    if(block_cb) cb_blocked_ = true;
  }
  return;
}

void ImageGrabber::show(bool depth, bool gpu, int wait){
  if(show_video_){
    if (gpu && !rgb_gim_.empty()){
      cv::namedWindow(name_ + " RGB",CV_WINDOW_OPENGL);
      cv::imshow(name_ + " RGB",rgb_gim_);
    }
    else if (!gpu && !rgb_im_.empty())
      cv::imshow(name_ + " RGB",rgb_im_);

    if (depth && gpu && !d_gim_.empty()){
      cv::namedWindow(name_ + " Depth",CV_WINDOW_OPENGL);
      cv::imshow(name_ + " Depth",d_gim_);
    }
    else if (depth && !gpu && !d_im_.empty())
      cv::imshow(name_ + " Depth",d_im_);
    
    cv::waitKey(wait);
  }

  if(pub_video_){
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_im_).toImageMsg();
    im_pub_.publish(image_msg);
  }
}

void ImageGrabber::rescale(){
  if(rgb_im_.size() != rgb_im_size_){
    cv::resize(rgb_im_, rgb_im_, rgb_im_size_);
    cv::cuda::resize(rgb_gim_, rgb_gim_, rgb_im_size_);
  }
  if(d_im_.size() != d_im_size_){
    cv::resize(d_im_, d_im_, d_im_size_);
    cv::cuda::resize(d_gim_, d_gim_, d_im_size_); 
  }
}

void ImageGrabber::mergeDepthFrames(int& indx){
  if (indx < depth_frames_.size()){
    depth_frames_[indx] = d_im_.clone();
    new_image_ = false;
  }
  else{
    int channels = d_im_.channels();
    int nRows = d_im_.rows;
    int nCols = d_im_.cols * channels;

    if(d_im_.isContinuous()){
      nCols *= nRows;
      nRows = 1;
    }

    uint16_t* p;
    std::vector<uint16_t*> p_;
    p_.reserve(depth_frames_.size());

    for(unsigned int i = 0; i < nRows && ros::ok(); i++){
      p = d_im_.ptr<uint16_t>(i);
      for(unsigned int k = 0; k < depth_frames_.size(); k++){
        p_[k] = depth_frames_[k].ptr<uint16_t>(i);
      }

      for(unsigned int j = 0; j < nCols && ros::ok(); j++){
        if(p[j] == 0){
          for(unsigned int k = depth_frames_.size() - 1; k > 0 && ros::ok(); k++){
            if(!depth_frames_[k].empty() && depth_frames_[k].isContinuous()){
              if (p_[k][j] != 0)
              p[j] = p_[k][j];
            }
          }
        }
      }
    }
    new_image_ = false;
  }
  indx++;
  return;
}

void ImageGrabber::colorBalance(double pct, bool cb_gpuMat){
  // http://www.morethantechnical.com/2015/01/14/simplest-color-balance-with-opencv-wcode/
  assert(rgb_im_.channels() == 3);
  assert(pct > 0 || pct < 100);

  std::vector<cv::Mat> split_im;
  cv::split(rgb_im_, split_im);

  for(int i = 0; i < 3; i++){
    cv::Mat flattened;
    split_im[i].reshape(1,1).copyTo(flattened);
    cv::sort(flattened, flattened, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    int min_val = flattened.at<uchar>(cvFloor(((float)flattened.cols) * pct / 200.0));
    int max_val = flattened.at<uchar>(cvCeil(((float)flattened.cols) * (1 - pct / 200.0)));

    // Saturate below the low percentile and above the high percentile
    split_im[i].setTo(min_val, split_im[i] < min_val);
    split_im[i].setTo(max_val, split_im[i] > max_val);

    // Scale the channel
    cv::normalize(split_im[i], split_im[i], 0, 255, cv::NORM_MINMAX);
  }

  cv::merge(split_im, rgb_im_);
  
  if(cb_gpuMat) rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
}

void ImageGrabber::createCVWindow(std::string window_name){
  cv::namedWindow(window_name);
  for(int i = 0 ; i < 5; i++)
    cv::waitKey(1);
  return;
}

void ImageGrabber::destroyCVWindow(std::string window_name){
  cv::destroyWindow(window_name);
  for(int i = 0 ; i < 5; i++)
    cv::waitKey(1);
  return;
}

void ImageGrabber::destroyAllCVWindows(){
  cv::destroyAllWindows(); 
  for(int i = 0 ; i < 5; i++)
    cv::waitKey(1);
  return;
}

void ImageGrabber::openVideoWriter(cv::VideoWriter &writer, std::string file_name){
  writer.open(file_name, CV_FOURCC('D','I','V','X'), 30, cv::Size(640,480));
  if (!writer.isOpened())
    std::cerr << "Could not open '" << file_name << "'" << std::endl;
  return;
}

void ImageGrabber::imCb(const sensor_msgs::Image::ConstPtr& msg){
  if(!cb_blocked_){
    try{
      rgb_im_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image;

      if(device_ == "Realsense" && color_balance_)
        colorBalance();

      rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
      //rescale();

      new_image_ = true;
      ROS_INFO("[ImageGrabber] Got new image");
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("[ImageGrabber] cv_bridge exception: %s", e.what());
    }
  }

  return;
}

void ImageGrabber::dualSubCb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg){
  if(!cb_blocked_){ 
    try{
      rgb_im_ = cv_bridge::toCvCopy ( rgb_msg, sensor_msgs::image_encodings::BGR8 ) -> image;
      d_im_ = cv_bridge::toCvCopy ( depth_msg, sensor_msgs::image_encodings::TYPE_16UC1 ) -> image;

      if(device_ == "Realsense" && color_balance_)
        colorBalance();

      rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
      d_gim_ = cv::cuda::GpuMat(d_im_);
      //rescale();

      new_image_ = true;
      ROS_INFO("[ImageGrabber] Got new image");
    }

    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  return;
}

void ImageGrabber::tripleSubCb (const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::PointCloud2ConstPtr& cloud){
  if(!cb_blocked_){
    try{
      rgb_im_ = cv_bridge::toCvCopy ( rgb_msg, sensor_msgs::image_encodings::BGR8 ) -> image; 
      d_im_ = cv_bridge::toCvCopy ( depth_msg, sensor_msgs::image_encodings::TYPE_16UC1 ) -> image;
      pcl::fromROSMsg(*cloud, cloud_);

      if(device_ == "Realsense" && color_balance_)
        colorBalance();

      rgb_gim_ = cv::cuda::GpuMat(rgb_im_);
      d_gim_ = cv::cuda::GpuMat(d_im_);

      //rescale();

      new_image_ = true;
      ROS_INFO("[ImageGrabber] Got new image");
    }
    
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  return;
}

void ImageGrabber::freeCb(){
  cb_blocked_ = false;
  return;
}

void ImageGrabber::blockCb(){
  cb_blocked_ = true;
  return;
}

void ImageGrabber::loadROSParams(std::string ns){

  // Topic Params
  nh_.param(ns + "/device", device_, device_);
  
  std::string prefix = "";
  if (device_ == "Realsense") prefix = "realsense_";
  else if (device_ == "Kinect") prefix = "kinect_";
  nh_.param(ns + "/topics" + "/" + prefix + "rgb", rgb_topic_, rgb_topic_);
  nh_.param(ns + "/topics" + "/" + prefix + "depth", depth_topic_, depth_topic_);
  nh_.param(ns + "/topics" + "/" + prefix + "pts", pts_topic_, pts_topic_);

  // Display params
  nh_.param(ns + "/display" + "/name", name_, name_);
  nh_.param(ns + "/display" + "/show", show_video_, show_video_);
  nh_.param(ns + "/display" + "/save", save_video_, save_video_);
  nh_.param(ns + "/display" + "/publish", pub_video_, pub_video_);

  // Image Size Params
  std::vector<int> temp = utils::createVecFromCVSize(rgb_im_size_);
  nh_.param(ns + "/image" + "/" + prefix + "rgb_resolution", temp, temp);
  rgb_im_size_ = utils::createCVSizeFromVec(temp);

  temp = utils::createVecFromCVSize(d_im_size_);
  nh_.param(ns + "/image" + "/" + prefix + "depth_resolution", temp, temp);
  d_im_size_ = utils::createCVSizeFromVec(temp);

  nh_.param(ns + "/image" + "/color_balance", color_balance_, color_balance_);

  ROS_INFO("[ImageGrabber] ROS Params Updated!");
}

void ImageGrabber::rqtCb(image_utils::GeneralRQTConfig &config, uint32_t level){
  if( level == 0 ){
    rgb_im_size_ = cv::Size(rgb_im_size_.width*config.im_scale, 
                              rgb_im_size_.height*config.im_scale);
    d_im_size_ = cv::Size(d_im_size_.width*config.im_scale,
                              d_im_size_.height*config.im_scale);

  }
  else if ( level == 1 ){
    if (config.im_size == 240){
      rgb_im_size_ = cv::Size(320,240);
      d_im_size_ = cv::Size(320,240);
    } 
    else if (config.im_size == 360){
      rgb_im_size_ = cv::Size(480,360);
      d_im_size_ = cv::Size(480,360);
    }
    else if (config.im_size == 480){
      rgb_im_size_ = cv::Size(640,480);
      d_im_size_ = cv::Size(640,480);
    } 
    else if (config.im_size == 720){
      rgb_im_size_ = cv::Size(1280,720);
      d_im_size_ = cv::Size(1280,720);
    } 
    else if (config.im_size == 1080){
      rgb_im_size_ = cv::Size(1920,1080);
      d_im_size_ = cv::Size(1920,1080);
    } 
  }

  ROS_INFO("[ImageGrabber] RQT: Rescale image to %dp",rgb_im_size_.height);
}
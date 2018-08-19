#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <social_robot/Utilities.h>
#include <social_robot/SilhouetteDetector.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>

#define foreach BOOST_FOREACH

int width = 640;
int height = 480;

SilhouetteDetector *sd;

std::vector<std::string> FilesToConvert;

std::string package_path;

std::string listofBagFiles = "bagfiles.txt";
std::string bagFileDir = "./decomp-bagfiles/";

std::string listofVideos = "videos.txt";
std::string videoDir = "./videos/";

void populateFilesToConvert()
{
  std::ifstream bagFile (listofBagFiles.c_str(), std::ifstream::in);
  std::ifstream videoFile (listofVideos.c_str(), std::ifstream::in);

  if (!bagFile)
  {
    std::cerr << "Could not access: " << listofBagFiles << std::endl;
    exit(1);
  }

  if (!videoFile)
  {
    std::cerr << "Could not access: " << listofVideos << std::endl;
    exit(1);
  }

  std::vector<std::string> bagList;
  std::vector<std::string> videoList;

  std::string filename, line;
  while (getline (bagFile,line))
  {
    std::stringstream liness(line);
    getline(liness,filename);
    bagList.push_back(filename);
  }

  while (getline (videoFile, line))
  {
    std::stringstream liness(line);
    getline(liness,filename);
    videoList.push_back(filename);

    std::cout << filename << std::endl;
  }

  for (unsigned int i = 0; i < bagList.size(); i++)
  {
    bool convert_rgb = true;
    bool convert_depth = true;

    std::cout << bagList[i] << std::endl;

    for (unsigned int j = 0; j < videoList.size(); j++)
    {
      if (bagList[i] == videoList[j])
        convert_rgb = false;

      if (bagList[i] + "_depth" == videoList[j])
        convert_depth = false;

      if ( !convert_rgb && !convert_depth)
        continue;
    }

    if (convert_rgb || convert_depth)
      FilesToConvert.push_back(bagList[i]);
  }

  return;
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "rgbwriter" );
  ros::NodeHandle nh;

  SilhouetteDetector temp_sd(nh); sd = &temp_sd;

  package_path = ros::package::getPath("social_robot");
  listofBagFiles = package_path + "/validation/" + listofBagFiles;
  listofVideos = package_path + "/validation/" + listofVideos;
  bagFileDir = package_path + "/validation/" + bagFileDir;
  videoDir = package_path + "/validation/" + videoDir;

  populateFilesToConvert();

  for (unsigned int i = 0; i < FilesToConvert.size(); i++)
    std::cout << FilesToConvert[i] << std::endl;

  for (unsigned int i = 0; i < FilesToConvert.size(); i++)
  {
    cv::VideoWriter rgbwriter;
    cv::VideoWriter dwriter;
    rosbag::Bag bag;

    ROS_INFO("Converting %S (Progress: %.2f)", FilesToConvert[i].c_str(), double(i)/double(FilesToConvert.size()));

    SD::head_angle = -7;
    SD::allowable_depth = {0,3.65};

    if (FilesToConvert[i].find("MIXED") != std::string::npos){
      SD::allowable_height = {-1,1};
      SD::head_angle = 0;
    }

    if (FilesToConvert[i].find("Combined") != std::string::npos){
      SD::allowable_height = {-0.4,1};
      SD::head_angle = 0;
      SD::allowable_depth = {0,5};
    }

    std::string bag_file = bagFileDir + FilesToConvert[i] + ".bag";
    std::string video_file = videoDir + FilesToConvert[i] + ".avi";
    std::string depth_video_file = videoDir + FilesToConvert[i] + "_depth.avi";

    bag.open(bag_file, rosbag::bagmode::Read);
    rgbwriter.open(video_file, CV_FOURCC('D','I','V','X'), 30, cv::Size(width,height));

    if (!rgbwriter.isOpened())
      std::cerr << "Could not open '" << video_file << "'" << std::endl;

    dwriter.open(depth_video_file, CV_FOURCC('D','I','V','X'), 30, cv::Size(width,height));

    if (!dwriter.isOpened())
      std::cerr << "Could not open '" << depth_video_file << "'" << std::endl;

    std::vector<std::string> topics(3);
    topics[0] = "/cam/rgb/image_raw";
    topics[1] = "/camera/rgb/image_color";
    topics[2] = "/camera/depth/image_raw";

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool rgb_exists = false;
    bool depth_exists = false;
    int frame_count = 0;

    cv::Mat depth_im, rgb_im;

    foreach(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == topics[0])
      {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
          rgb_im = cv_bridge::toCvCopy ( s, enc::BGR8 )->image;
          rgb_exists = true;
        }
      }

      else if (m.getTopic() == topics[1])
      {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
          rgb_im = cv_bridge::toCvCopy ( s, enc::BGR8 )->image;
          rgb_exists = true;
        }
      }

      else if (m.getTopic() == topics[2])
      {
        sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
        if (i != NULL)
        {
          depth_im = cv_bridge::toCvCopy( i, enc::TYPE_16UC1) -> image;
          depth_exists = true;
        }
      }

      if (rgb_exists && depth_exists)
      {
        frame_count ++;

        cv::Mat disp_im;

        sd->preprocess(depth_im,disp_im);

        if (rgbwriter.isOpened())
          rgbwriter << rgb_im;

        cv::cvtColor(disp_im,disp_im,CV_GRAY2RGB);

        if (dwriter.isOpened())
          dwriter << disp_im;

        rgb_exists = false;
        depth_exists = false;
      }  
    }

    bag.close();
    rgbwriter.release();
    dwriter.release();
    cv::destroyAllWindows();
    cv::waitKey(1);
  }

  return 0;
}


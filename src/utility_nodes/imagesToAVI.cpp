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

std::string listofBagFiles = "image-bagfiles.txt";
std::string bagFileDir = "./image-bagfiles/";

std::string videoName = "combined_rgb.avi";
std::string depthVideoName = "combined_depth.avi";
std::string bagName = "combined.bag";

void populateFilesToConvert()
{
  std::ifstream bagFile (listofBagFiles.c_str(), std::ifstream::in);

  if (!bagFile)
  {
    std::cerr << "Could not access: " << listofBagFiles << std::endl;
    exit(1);
  }

  std::string filename, line;
  while (getline (bagFile,line))
  {
    std::stringstream liness(line);
    getline(liness,filename);
    FilesToConvert.push_back(filename);
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
  bagFileDir = package_path + "/validation/" + bagFileDir;
  std::string video_file = package_path + "/validation/" + videoName;
  std::string depth_video_file = package_path + "/validation/" + depthVideoName;
  std::string combined_bag_file = package_path + "/validation/" + bagName;

  populateFilesToConvert();

  cv::VideoWriter rgbwriter;
  cv::VideoWriter dwriter;
  
  rgbwriter.open(video_file, CV_FOURCC('D','I','V','X'), 30, cv::Size(width,height));

  if (!rgbwriter.isOpened())
    std::cerr << "Could not open '" << video_file << "'" << std::endl;

  dwriter.open(depth_video_file, CV_FOURCC('D','I','V','X'), 30, cv::Size(width,height));

  if (!dwriter.isOpened())
    std::cerr << "Could not open '" << depth_video_file << "'" << std::endl;

  rosbag::Bag combined_bag;
  combined_bag.open(combined_bag_file, rosbag::bagmode::Write);


  for (unsigned int i = 0; i < FilesToConvert.size(); i++)
  {
    rosbag::Bag bag;

    ROS_INFO("Converting %S (Progress: %.2f)", FilesToConvert[i].c_str(), double(i)/double(FilesToConvert.size()));

    std::string bag_file = bagFileDir + FilesToConvert[i] + ".bag";
    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics(6);
    topics[0] = "/cam/rgb/image_raw";
    topics[1] = "/camera/rgb/image_color";
    topics[2] = "/camera/depth/image_raw";
    topics[3] = "/head_xtion/depth/image";
    topics[4] = "/head_xtion/rgb/image_color";
    topics[5] = "/pedestrian_tracking/pedestrian_array";

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

      else if (m.getTopic() == topics[3])
      {
        sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
        if (i != NULL)
        {
          depth_im = cv_bridge::toCvCopy(i, enc::TYPE_16UC1) -> image;
          depth_exists = true;
        }
      }

      else if (m.getTopic() == topics[4])
      {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
          rgb_im = cv_bridge::toCvCopy(s, enc::BGR8) -> image;          
          rgb_exists = true;
        }
      }

      else if (m.getTopic() == topics[5])
      {
        // strands_perception_people_msgs::PedestrianTrackingArray::ConstPtr msg = m.instantiate<strands_perception_people_msgs::PedestrianTrackingArray>();
        // if (msg != NULL)
        // {

        // }
      }

      if (rgb_exists && depth_exists)
      {
        frame_count ++;

        sensor_msgs::Image image_msg;
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_im).toImageMsg(image_msg);
        combined_bag.write("/cam/rgb/image_raw",ros::Time::now(),image_msg);

        cv_bridge::CvImage(std_msgs::Header(), enc::TYPE_16UC1, depth_im).toImageMsg(image_msg);
        combined_bag.write("/camera/depth/image_raw",ros::Time::now(),image_msg);

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
    cv::destroyAllWindows();
    cv::waitKey(1);
  }

  rgbwriter.release();
  dwriter.release();
  combined_bag.close();

  return 0;
}
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <social_robot/Utilities.h>
#include <social_robot/SilhouetteDetector.h>
#include <social_robot/StandaloneRPS.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>

#define foreach BOOST_FOREACH

int width = 640;
int height = 480;

SilhouetteDetector *sd;
StandaloneRPS *SRPS;

std::vector<std::string> FilesToConvert;

std::string package_path;

std::string listofBagFiles = "bagfiles.txt";
std::string bagFileDir = "./decomp-bagfiles/";

std::string listOfImgs = "images.txt";
std::string imgDir = "./images/";
std::string mImgDir = "./images-merged/";

// Video ROS parameters
int frames_to_check = 1; //7
int frames_to_merge = 5; //10

std::vector<cv::Mat> depth_frames(frames_to_merge-1);

void populateFilesToConvert()
{
  std::ifstream bagFile (listofBagFiles.c_str(), std::ifstream::in);
  std::ifstream imgFile (listOfImgs.c_str(), std::ifstream::in);

  if (!bagFile)
  {
    std::cerr << "Could not access: " << listofBagFiles << std::endl;
    exit(1);
  }

  if (!imgFile)
  {
    std::cerr << "Could not access: " << listOfImgs << std::endl;
    exit(1);
  }

  std::vector<std::string> bagList;
  std::vector<std::string> imgList;

  std::string filename, line;
  while (getline (bagFile,line))
  {
    std::stringstream liness(line);
    getline(liness,filename);
    bagList.push_back(filename);
  }

  while (getline (imgFile, line))
  {
    std::stringstream liness(line);
    getline(liness,filename);
    imgList.push_back(filename);

    std::cout << filename << std::endl;
  }

  for (unsigned int i = 0; i < bagList.size(); i++)
  {
    bool convert = true;

    std::cout << bagList[i] << std::endl;

    for (unsigned int j = 0; j < imgList.size(); j++)
    {
      if (bagList[i] == imgList[j])
        convert = false;

      if (!convert)
        continue;
    }

    if (convert)
      FilesToConvert.push_back(bagList[i]);
  }

  return;
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "imgwriter" );
  ros::NodeHandle nh;

  SilhouetteDetector temp_sd(nh); sd = &temp_sd;
  StandaloneRPS temp_SRPS(nh); SRPS = &temp_SRPS;

  package_path = ros::package::getPath("social_robot");
  listofBagFiles = package_path + "/validation/" + listofBagFiles;
  listOfImgs = package_path + "/validation/" + listOfImgs;
  bagFileDir = package_path + "/validation/" + bagFileDir;
  imgDir = package_path + "/validation/" + imgDir;
  mImgDir = package_path + "/validation/" + mImgDir;

  populateFilesToConvert();

  for (unsigned int i = 0; i < FilesToConvert.size(); i++)
    std::cout << FilesToConvert[i] << std::endl;

  for (unsigned int i = 0; i < FilesToConvert.size(); i++)
  {
    ROS_INFO("Converting %S (Progress: %.2f)", FilesToConvert[i].c_str(), double(i)/double(FilesToConvert.size()));
    rosbag::Bag bag;

    SD::head_angle = -7;
    SD::allowable_depth = {0,3.65};

    if (FilesToConvert[i].find("MIXED") != std::string::npos){
      SD::allowable_height = {-1,1};
      SD::head_angle = 0;
    }

    if (FilesToConvert[i].find("Combined") != std::string::npos){
      continue;
      // SD::allowable_height = {-0.4,1};
      // SD::head_angle = 0;
      // SD::allowable_depth = {0,5};
    }

    std::string bag_file = bagFileDir + FilesToConvert[i] + ".bag";
    std::string rgb_file = imgDir + FilesToConvert[i] + ".jpg";
    std::string depth_file = imgDir + FilesToConvert[i] + ".tiff";
    std::string m_rgb_file = mImgDir + FilesToConvert[i] + ".jpg";
    std::string m_depth_file = mImgDir + FilesToConvert[i] + ".tiff";

    bag.open(bag_file, rosbag::bagmode::Read);
    
    std::vector<std::string> topics(3);
    topics[0] = "/cam/rgb/image_raw";
    topics[1] = "/camera/rgb/image_color";
    topics[2] = "/camera/depth/image_raw";

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool rgb_exists = false;
    bool depth_exists = false;
    int loop_ctr =0;
    int frames_checked = 0;

    cv::Mat depth_im, rgb_im;

    foreach(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == topics[0])
      {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
          rgb_im.release();
          rgb_im = cv_bridge::toCvCopy ( s, enc::BGR8 )->image;
          rgb_exists = true;
        }
      }

      else if (m.getTopic() == topics[1])
      {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
          rgb_im.release();
          rgb_im = cv_bridge::toCvCopy ( s, enc::BGR8 )->image;
          rgb_exists = true;
        }
      }

      else if (m.getTopic() == topics[2])
      {
        sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
        if (i != NULL)
        {
          depth_im.release();
          depth_im = cv_bridge::toCvCopy( i, enc::TYPE_16UC1) -> image;
          depth_exists = true;
        }
      }

      if (rgb_exists && depth_exists)
      {
        depth_exists = false;
        rgb_exists = false;

        if (frames_checked >= frames_to_check)
          break;

        loop_ctr++;

        if (frames_to_merge > 1 && loop_ctr < frames_to_merge)
        {
          depth_frames[loop_ctr-1] = depth_im.clone();
          continue;
        }

        frames_checked++;
        loop_ctr = 0;

        cv::imwrite(rgb_file, rgb_im);
        cv::imwrite(depth_file, depth_im);

        if (frames_to_merge > 1)
        {
          SRPS->mergeDepthFrames(depth_frames,depth_im);
          depth_frames.clear(); depth_frames.resize(frames_to_merge-1);
        }

        cv::imwrite(m_rgb_file, rgb_im);
        cv::imwrite(m_depth_file, depth_im);

        ros::Rate r(100);
        r.sleep();

      }  
    }

    bag.close();
    cv::destroyAllWindows();
    cv::waitKey(1);
  }

  return 0;
}


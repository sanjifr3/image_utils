#ifndef PCL_UTILITIES_H
#define PCL_UTILITIES_H

// TF libraries
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

// // Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

// Social robot libraries
#include <image_utils/Utilities.h>

class PCLUtilities
{
  public:
  	PCLUtilities ( ros::NodeHandle nh );
  	~PCLUtilities ( void );

    std::vector<cv::Point3f> getHeadCoordinatesinMapFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates);
    std::vector<cv::Point3f> getCoordinatesinBaseFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates);
  	std::vector<cv::Point3f> getCoordinatesinMapFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates);

  	std::vector<cv::Point2f> getPointsToCheck(cv::Rect &potential);
  	std::vector<cv::Point2f> getPointsToCheck2(cv::Rect &potential);
    cv::Point3f getCoordinate(pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Rect &rect);
  	std::vector<cv::Point3f> getCoordinateinBaseFrame(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect &potential);
  	bool isGoodCoordinate(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Point3f &coordinate, cv::Rect potential, double min, double max);

  	std::vector<cv::Point3f> getCoordinateinBaseFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect &potential, std::vector<cv::Point2f> &points_to_check);
  	bool isGoodCoordinate(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<bool> &is_good_coord, std::vector<cv::Point2f> &im_coord, std::vector<cv::Point3f> &coord, cv::Point3f &avg_coord, cv::Rect potential, double min, double max);

  	cv::Point3f frameTransform ( cv::Point3f current_frame_coordinates, std::string frameFrom, std::string frameTo );
  	void getMinMaxPCL (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Point3f &min, cv::Point3f &max);
  	Rpose getRobotPose (std::string frame);
  	
  	cv::Point3f getAvgCoord(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect& rect);
  		
  private:
  	geometry_msgs::PoseStamped createPose ( std::string frame, double x, double y, double z, double ax, double ay, double az );
  	geometry_msgs::PoseStamped transformPose ( std::string frameTo, geometry_msgs::PoseStamped poseFrom ); 
  	std::vector<cv::Point3f> getBatchCoordinates(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Point2f> &points_to_check);
    void loadROSParams(std::string ns=ros::this_node::getName());

  public:  
    std::string device_ = "Realsense"; // "Kinect";
    std::vector<int> im_resolution_ = {1280,720}; // {640,480};
  
  private:
  	ros::NodeHandle nh_;
    tf::TransformListener tfListener_;

    std::string kinect_frame_ = "camera_depth_optical_frame";
    std::string base_frame_ = "base_link";
    std::string map_frame_ = "map";

    double head_offset_ = 0;//0.20;
    double kinect_cloud_shift_ = 0;//30;
    double blueberry_cloud_shift_ = 0;//100;
    double avg_coord_granularity_ = 2; 
};

#endif //PCL_UTILITIES_H

#ifndef UTILITIES_H
#define UTILITIES_H

// ROS Libraries
#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include <list>
#include <memory>
#include <algorithm>
#include <omp.h>
#include <unistd.h>
#include <cstdlib>
#include <sys/time.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/select.h>
#include <boost/lexical_cast.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/photo/photo.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/cuda.hpp>
namespace enc = sensor_msgs::image_encodings;

namespace RPS
{
  struct personInfo{
    personInfo() {}
    personInfo(cv::Rect l_, cv::Scalar col_, std::string n_, double dc_, double rc_) : 
                    loc(l_), color(col_), name(n_), det_conf(dc_), rec_conf(rc_) {}    
    personInfo(cv::Rect l_, cv::Scalar col_, std::string n_, double rc_) : 
                    loc(l_), color(col_), name(n_), rec_conf(rc_) {}
    // Location
    cv::Rect loc = cv::Rect(0,0,0,0);
    cv::Point3f coord = cv::Point3f(0,0,0);
    double dist = 0.0;
    double det_conf = 1.0;

    // Identification
    std::string name = "Potential";
    double rec_conf = -0.05;
    
    // Orientation    
    std::string orient = "B";
    double angle = -180;
    cv::Scalar color = CV_RGB(255,255,255);
    
    // Counters
    int g_d_ctr = 0; // Global detection counter
    int d_ctr = 1; // Detection counter
    std::vector<int> ctr = {0,0,0};

    std::vector<cv::Point> jawPts, rEyeBrowPts, lEyeBrowPts, nosePts, 
                             rEyePts, lEyePts, mouthPts;

    //int frame_ctr = 1; // Frame counter
    /*int f_ctr = 0; // Face counter
    int l_ctr = 0; // Left profile counter
    int r_ctr = 0; // Right profile counter*/
  };

  struct objectInfo{
    std::string name = "";
    int id = -1;
    double det_conf = 0.0;
    
    int state = 0;
    double rec_conf = 0.0;
    
    cv::Rect loc = cv::Rect(0,0,0,0);
    cv::Point3f coord = cv::Point3f(0,0,0);
    double dist = 0.0;
    cv::Scalar color;
    int d_ctr = 1;
  };

  struct desiredPos{
    cv::Point2f coordinate;
    double angle;
    double distance;
  };
}

struct Rpose{
  double x;
  double y;
  double th;
};

struct Silhouettes{
  std::string temp_name;
  double corr;
  double scale;
  double occupied;
  cv::Rect loc;
};

namespace utils{
  template <typename T> 
  std::string to_string (const T& n){
    std::ostringstream stm;
    stm << n;
    return stm.str();
  }
  
  template <typename Out> 
  void split(const std::string &s, char delim, Out result){
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) 
    {
      *(result++) = item;
    }
  }
  
  template <typename T> 
  std::string NumToStr (T Number){
    std::stringstream ss;
    ss << Number;
    return ss.str();
  }
  
  template <typename T> 
  T StrToNum (const std::string &Text){                                           
    std::stringstream ss(Text);
    T result;
    return ss >> result ? result : 0;
  }
 
  std::vector<std::string> split(const std::string &s, char delim);
  std::string double2Str (double num, int num_sig_digs = 0);

  // Angle Manipulation
  double degToRad (double angle);
  double radToDeg (double angle);
  double modTo360 (double angle);
  double modTo180 (double angle);

  // Rectangle Manipulation
  cv::Point2f getCenter (cv::Rect rect);
  cv::Rect searchToGlobalFrame ( cv::Rect global_frame, cv::Rect face );
  cv::Rect doubleSize (cv::Rect original_size, int cols = 640, int rows = 480);
  cv::Rect resize (cv::Rect original, int delta_x = 0, int delta_y = 0, int cols = 640, int rows = 480);
  cv::Rect rescale (cv::Rect original, double scale_x = 2, double scale_y = 0, int cols = 640, int rows = 480);
  std::vector<cv::Rect> mergeRects (std::vector<cv::Rect> rects);
  
  // Matrix Manipulation
  void translateMat (cv::Mat &img, int offsetx, int offsety);
  
  // Distance Calculators
  template<typename T> 
  double euclideanDistance (T p1, T p2){
    return sqrt ( pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
  }
  //double euclideanDistance (cv::Point2f p1, cv::Point2f p2);
  //double euclideanDistance (cv::Point3f p1, cv::Point3f p2);
  double euclideanDistance (cv::Rect r1, cv::Rect r2);
  double diag (cv::Rect r);
  
  // Vector Calculators
  void getMin (std::vector<cv::Point2f> vectr, double &min_x, double &min_y);
  void getMax (std::vector<cv::Point2f> vectr, double &max_x, double &max_y);
  ptrdiff_t findInVector (int id, std::vector<int> vectr);
  ptrdiff_t findInVector (std::string str, std::vector<std::string> vectr);
  int sumVec (std::vector<int>& vec);

  std::vector<int> createVecFromCVSize (cv::Size);
  cv::Size createCVSizeFromVec (std::vector<int> vec);
  
  template <typename T> 
  std::vector<T> createVector(T data[]){
    std::vector<T> vect (data, data + sizeof(data) / sizeof (data[0]) );
    return vect;
  }

  template <typename T> 
  std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b){
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::plus<T>());
    return result;
  }

/*
  template <typename T> 
  std::list<T> toList(std::vector<T> source){
    return new std::list<T>(source);
  }
*/
    
  // I/O Functions
  //std::string getResponse(std::string question, __time_t time_limit = 15);

  // Quartile Calculator
  template<typename T>
  static inline double Lerp(T v0, T v1, T t){
    return (1 - t)*v0 + t*v1;
  }

  template<typename T>
  static inline std::vector<T> Quantile(const std::vector<T>& inData, const std::vector<T>& probs){
    if (inData.empty())
      return std::vector<T>();

    if (1 == inData.size())
      return std::vector<T>(1, inData[0]);

    std::vector<T> data = inData;
    std::sort(data.begin(), data.end());
    std::vector<T> quantiles;

    for (size_t i = 0; i < probs.size(); ++i){
      T poi = Lerp<T>(-0.5, data.size() - 0.5, probs[i]);

      size_t left = std::max(int64_t(std::floor(poi)), int64_t(0));
      size_t right = std::min(int64_t(std::ceil(poi)), int64_t(data.size() - 1));

      T datLeft = data.at(left);
      T datRight = data.at(right);

      T quantile = Lerp<T>(datLeft, datRight, poi - left);

      quantiles.push_back(quantile);
    }

    return quantiles;
  }
  
  // Time
  double getWallTime();
  void printFPS();

  // Update RPS
  void updateStoredPpl(std::vector<RPS::personInfo>& people, std::vector<cv::Rect>& new_people, 
                              cv::Scalar color = CV_RGB(255,255,255), std::string name="Potential", 
                              double conf=-0.05);
  void updateStoredPpl(std::list<RPS::personInfo>& people, std::vector<cv::Rect>& new_people, 
                              cv::Scalar color = CV_RGB(255,255,255), std::string name="Potential", 
                              double conf=-0.05);
  
  double time_ = 0;
  double im_dist_tol_ = 50;
  double dist_tol_ = 0.5;
}

#endif // UTILITIES

#include <image_utils/Utilities.h>

namespace utils{
  /////////////////////////
  // String Manipulation //
  /////////////////////////

  std::vector<std::string> split(const std::string &s, char delim){
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
  }

  std::string double2Str (double num, int num_sig_digs){
    assert (num_sig_digs >= 0);

    std::string str = boost::lexical_cast<std::string>(num);

    std::size_t found = str.find(".");

    if (found != std::string::npos){
      if (num_sig_digs == 0) num_sig_digs = -1;
      return str.substr(0,found+num_sig_digs+1);
    }

    return str;
  }

  ////////////////////////
  // Angle Manipulation //
  ////////////////////////  

  double degToRad (double angle){
    double angle_rad = angle*M_PI/180.0;
    return angle_rad;
  }

  double radToDeg (double angle){
    double angle_deg = angle*180.0/M_PI;
    return angle_deg;
  }

  double modTo360 (double angle){
    while ( angle > 360 || angle < 0 ){
      if ( angle < 0 )
        angle = 360 + angle;

      else if ( angle > 360 )
        angle = angle - 360;
    }

    return angle;
  }

  double modTo180 (double angle){
    while ( angle < -180 || angle > 180 ){
      if ( angle > 180 )
        angle = angle - 360;

      else if ( angle < -180 )
        angle = angle + 360;
    }

    return angle;
  }

  ////////////////////////////
  // Rectangle Manipulation //
  ////////////////////////////

  cv::Point2f getCenter (cv::Rect rect){
    return cv::Point2f(rect.x + rect.width/2, rect.y + rect.height/2);
  }

  cv::Rect searchToGlobalFrame ( cv::Rect global_frame, cv::Rect face ){
    cv::Rect face_in_global_frame;

    face_in_global_frame.x = global_frame.x + face.x;
    face_in_global_frame.y = global_frame.y + face.y;
    face_in_global_frame.width = face.width;
    face_in_global_frame.height = face.height;

    return face_in_global_frame;
  }

  cv::Rect doubleSize (cv::Rect original_size, int cols, int rows){
    cv::Rect new_size;

    new_size.x = original_size.x - original_size.width/2 - 10;
    if ( new_size.x < 0 ) new_size.x = 0;

    new_size.y = original_size.y - original_size.height/2;
    if ( new_size.y < 0 ) new_size.y = 0;

    new_size.width = original_size.width*2.5 + 10;
    if ( ( new_size.x + new_size.width ) > cols ) new_size.width = cols - new_size.x;

    new_size.height = original_size.height*2.5;
    if ( ( new_size.y + new_size.height ) > rows ) new_size.height = rows - new_size.y;

    return new_size;
  }

  cv::Rect resize (cv::Rect original, int delta_x, int delta_y, int cols, int rows){
    if ( delta_x == 0 )
      delta_x = original.width/2;
    if ( delta_y == 0 )
      delta_y = original.height/2;

    cv::Rect new_rect = original;

    cv::Point inflationPoint (-delta_x, -delta_y);
    cv::Size inflationSize (delta_x, delta_y);
    
    new_rect += inflationPoint;
    new_rect += inflationSize;

    if (new_rect.x < 0) new_rect.x = 0;
    if (new_rect.y < 0) new_rect.y = 0;
    if ((new_rect.x + new_rect.width) > cols)
      new_rect.width = cols - new_rect.x;
    if ((new_rect.y + new_rect.height) > rows)
      new_rect.height = rows - new_rect.y;
    return new_rect;
  }

  cv::Rect rescale (cv::Rect original, double scale_x, double scale_y, int cols, int rows){
    if (scale_x == 0){
      scale_x = 2;
      scale_y = 2;
    }

    else if ( scale_y == 0)
      scale_y = scale_x;

    cv::Point center = getCenter(original);

    int width = (int)(original.width*scale_x);
    int height = (int)(original.height*scale_y);
    int x = center.x - width/2;
    int y = center.y - height/2;

    if ( x < 0 ) x = 0;
    if ( y < 0 ) y = 0;
    if ( (x + width) > cols ) width = cols - x;
    if ( (y + height) > rows ) height = rows - y;

    return cv::Rect (x,y,width,height);
  }

  std::vector<cv::Rect> mergeRects (std::vector<cv::Rect> rects){
    std::vector<cv::Rect> merged;

    for(unsigned int i = 0; i < rects.size(); i++){
      bool associated = false;
      for(unsigned int j = 0; j < merged.size(); j++){
        if(merged[j].x < rects[i].x + rects[i].width &&
           merged[j].x + merged[j].width > rects[i].x &&
           merged[j].y + merged[j].height > rects[i].y &&
           merged[j].y < rects[i].y + rects[i].height){
            associated = true;
            int min_x = std::min(merged[j].x, rects[i].x);
            int min_y = std::min(merged[j].y, rects[i].y);
            int max_x = std::max(merged[j].x + merged[j].width, rects[i].x + rects[i].width);
            int max_y = std::max(merged[j].y + merged[j].height, rects[i].y + rects[i].height);
            merged[j] = cv::Rect(min_x, min_y, (max_x-min_x), (max_y-min_y));
            break;
        }
      }

      if(!associated){
        merged.push_back(rects[i]);
      }
    }
    return merged;
  }

  /////////////////////////
  // Matrix Manipulation //
  /////////////////////////  

  void translateMat (cv::Mat &img, int offsetx, int offsety){
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    cv::warpAffine(img,img,trans_mat,img.size());
  }

  //////////////////////////
  // Distance Calculators //
  //////////////////////////  

  double euclideanDistance (cv::Rect r1, cv::Rect r2){
    cv::Point2f r1_center = cv::Point2f(r1.x + r1.width/2, r1.y + r1.height/2);
    cv::Point2f r2_center = cv::Point2f(r2.x + r2.width/2, r2.y + r2.height/2);

    return euclideanDistance(r1_center,r2_center);
  }

  double diag (cv::Rect r){
    return sqrt((r.width*r.width) + (r.height*r.height));
  }

  ////////////////////////
  // Vector Calculators //
  ////////////////////////

  void getMin (std::vector<cv::Point2f> vectr, double &min_x, double &min_y){
    min_x = 100000000;
    min_y = 100000000;
    for (std::size_t i = 0; i < vectr.size(); i++){
      if ( vectr[i].x < min_x )
        min_x = vectr[i].x;

      if ( vectr[i].y < min_y )
        min_y = vectr[i].y;
    }

    return;
  }

  void getMax (std::vector<cv::Point2f> vectr, double &max_x, double &max_y){
    max_x = -100000000;
    max_y = -100000000;
    for (std::size_t i = 0; i < vectr.size(); i++){
      if ( vectr[i].x > max_x )
        max_x = vectr[i].x;

      if ( vectr[i].y > max_y )
        max_y = vectr[i].y;
    }

    return;
  }

  ptrdiff_t findInVector (int id, std::vector<int> vectr){
    ptrdiff_t pos = find(vectr.begin(), vectr.end(), id) - vectr.begin();

    if ( pos < vectr.size() )
      return pos;
    else
      return -1;
  }

  ptrdiff_t findInVector (std::string str, std::vector<std::string> vectr){
    ptrdiff_t pos = find(vectr.begin(), vectr.end(), str) - vectr.begin();

    if ( pos < vectr.size() )
      return pos;
    else
      return -1;
  }

  int sumVec (std::vector<int>& vec){
    return std::accumulate(vec.begin(), vec.end(), 0);
  }

  double getWallTime(){
  struct timeval time;
    if (gettimeofday(&time, NULL)) {
      return 0;
    }
    return (double) time.tv_sec + (double) time.tv_usec * .000001;
  }

  void printFPS(){
    std::cout << "FPS " << 1./(getWallTime() - time_ + 0.00000001) << std::endl;
    time_ = getWallTime();
    return;
  }

  std::vector<int> createVecFromCVSize (cv::Size sz){
    std::vector<int> temp = {sz.width,sz.height};
    return temp;
  }

  cv::Size createCVSizeFromVec (std::vector<int> vec){
    assert (vec.size() == 2);
    return cv::Size(vec[0],vec[1]);
  }


  // std::vector<double> createVector(double data[])
  // {
  //   std::vector<double> vect (data, data + sizeof(data) / sizeof (data[0]) );
  //   return vect;
  // }

  ///////////////////
  // I/O Functions //
  ///////////////////

  /*
  std::string getResponse(std::string question, __time_t time_limit)
  {
    if (question != "")
      std::cout << question << std::endl;
    std::string response = "";

    fd_set readSet; 
    FD_ZERO(&readSet);
    FD_SET(STDIN_FILENO, &readSet);
    struct timeval tv = {time_limit,0}; // 10 seconds, 0 microseconds
    if (select(STDIN_FILENO+1, &readSet, NULL, NULL, &tv) < 0) perror ("select");

    bool got_response = (FD_ISSET(STDIN_FILENO, &readSet)) ? (std::cin >> response) : false;

    if(!got_response)
      std::cout << "No Response!" << std::endl;

    return response;
  }*/
  void updateStoredPpl(std::vector<RPS::personInfo>& people, std::vector<cv::Rect>& new_people, 
                              cv::Scalar color, std::string name, double conf){
    for(unsigned int i = 0; i < new_people.size(); i++){
      cv::Point2f new_target = getCenter(new_people[i]);
      bool associated = false;
      for(unsigned int j = 0; j < people.size(); j++){
        cv::Point2f stored_target = getCenter(people[j].loc);
        if(euclideanDistance(new_target,stored_target) < im_dist_tol_){
          associated = true;

          people[j].loc = new_people[i];
          people[j].d_ctr += 1;
        }
      }
      if(!associated)
        people.push_back(RPS::personInfo(new_people[i], color, name, conf));
    }
  }

  void updateStoredPpl(std::list<RPS::personInfo>& people, std::vector<cv::Rect>& new_people,
                              cv::Scalar color, std::string name, double conf){
    for(unsigned int i = 0; i < new_people.size(); i++){
      cv::Point2f new_target = getCenter(new_people[i]);
      bool associated = false;
      for(std::list<RPS::personInfo>::iterator ptr = people.begin(); ptr != people.end(); ptr++){
        cv::Point2f stored_target = getCenter(ptr->loc);
        if(euclideanDistance(new_target, stored_target) < im_dist_tol_){
          associated = true;
          ptr->loc = new_people[i];
          ptr->d_ctr += 1;
        }
      }

      if(!associated)
        people.push_back(RPS::personInfo(new_people[i], color, name, conf));
    }
  }
}
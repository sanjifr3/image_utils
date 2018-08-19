#include <image_utils/PCLUtilities.h>

PCLUtilities::PCLUtilities ( ros::NodeHandle nh ){
  this->nh_ = nh;
  loadROSParams();
  ROS_INFO ( "[PCLUtilities] Intialized!");
}

PCLUtilities::~PCLUtilities ( void ){
}

std::vector<cv::Point3f> PCLUtilities::getHeadCoordinatesinMapFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates){
  std::vector<cv::Point3f> coordinates(px_coordinates.size());

  if (coordinates.size() == 0)
    return coordinates;

  for ( unsigned int i = 0; i < px_coordinates.size(); i++ ){
    cv::Point2f face_center (px_coordinates[i].x + px_coordinates[i].width/2, px_coordinates[i].y + px_coordinates[i].height/2);
    unsigned int new_y = 0;
    unsigned int new_x = 0;
    
    if ( device_ == "Kinect" || device_ == "Realsense" ){
      new_y = face_center.y + kinect_cloud_shift_;
      new_x = face_center.x;
    }

    else if ( device_ == "Blueberry" ){
      new_y = face_center.y + blueberry_cloud_shift_;
      new_x = face_center.x + 10;
    }

    else
      ROS_ERROR("[PCLUtilities] Invalid device type given: %s", device_.c_str());

    if ( new_x < 0 ) new_x = 0 + 10;
    if ( new_x > im_resolution_[0] ) new_x = im_resolution_[0] - 15;

    if ( new_y < 0 ) new_y = 0 + 10;
    if ( new_y > im_resolution_[1] ) new_y = im_resolution_[1] - 15 ;

    if ( new_x >= pt_cloud.width ){
      std::cout << "ERROR: new_x exceed pt_cloud size" << std::endl;
      std::cout << "new_x:  " << new_x << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    if ( new_y >= pt_cloud.height ){
      std::cout << "ERROR: new_y exceed pt_cloud size" << std::endl;
      std::cout << "new_y: " << new_y << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    int old_y = new_y;

    do{
      coordinates[i].x = pt_cloud.at(new_x, new_y).x;
      coordinates[i].y = pt_cloud.at(new_x, new_y).y;
      coordinates[i].z = pt_cloud.at(new_x, new_y).z;
      new_y++;
    } while (std::isnan(coordinates[i].x) && new_y < pt_cloud.height);

    //std::cout << "PCLUtilities: Had to look " << new_y - old_y + 1 << " pixels to find coordinate" << std::endl;

    if ( device_ == "Blueberry" )
      coordinates[i] = frameTransform ( coordinates[i], "kinect", "map");
  }

  return coordinates;
}

std::vector<cv::Point3f> PCLUtilities::getCoordinatesinBaseFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates){
  std::vector<cv::Point3f> coordinates(px_coordinates.size());

  if (coordinates.size() == 0)
    return coordinates;

  for ( unsigned int i = 0; i < px_coordinates.size(); i++ ){
    cv::Point2f face_center (px_coordinates[i].x + px_coordinates[i].width/2, px_coordinates[i].y + px_coordinates[i].height/2);
    unsigned int new_y = 0;
    unsigned int new_x = 0;
    
    if ( device_ == "Kinect" || device_ == "Realsense"){
      new_y = face_center.y + kinect_cloud_shift_;
      new_x = face_center.x;
    }

    else if ( device_ == "Blueberry" ){
      new_y = face_center.y + blueberry_cloud_shift_;
      new_x = face_center.x + 10;
    }

    else
      ROS_ERROR("[PCLUtilities] Invalid device type given: %s", device_.c_str());

    if ( new_x < 0 ) new_x = 0 + 10;
    if ( new_x > im_resolution_[0] ) new_x = im_resolution_[0] - 15;

    if ( new_y < 0 ) new_y = 0 + 10;
    if ( new_y > im_resolution_[1] ) new_y = im_resolution_[1] - 15 ;

    if ( new_x >= pt_cloud.width ){
      std::cout << "ERROR: new_x exceed pt_cloud size" << std::endl;
      std::cout << "new_x:  " << new_x << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    if ( new_y >= pt_cloud.height ){
      std::cout << "ERROR: new_y exceed pt_cloud size" << std::endl;
      std::cout << "new_y: " << new_y << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    int old_y = new_y;

    do{
      coordinates[i].x = pt_cloud.at(new_x, new_y).x;
      coordinates[i].y = pt_cloud.at(new_x, new_y).y;
      coordinates[i].z = pt_cloud.at(new_x, new_y).z;
      new_y++;
    } while (std::isnan(coordinates[i].x) && new_y < pt_cloud.height);

    // std::cout << "PCLUtilities: Had to look " << new_y - old_y + 1 << " pixels to find coordinate" << std::endl;

    if ( device_ == "Blueberry" )
      coordinates[i] = frameTransform ( coordinates[i], "kinect", "base");
  }

  return coordinates;
}

std::vector<cv::Point3f> PCLUtilities::getCoordinatesinMapFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Rect> &px_coordinates){
  std::vector<cv::Point3f> coordinates(px_coordinates.size());

  for ( unsigned int i = 0; i < px_coordinates.size(); i++ ){
    cv::Point2f face_center (px_coordinates[i].x + px_coordinates[i].width/2, px_coordinates[i].y + px_coordinates[i].height/2);
    unsigned int new_y = 0;
    unsigned int new_x = 0;
    
    if ( device_ == "Kinect" || device_ == "Realsense" ){
      new_y = face_center.y + kinect_cloud_shift_;
      new_x = face_center.x;
    }

    else if ( device_ == "Blueberry" ){
      new_y = face_center.y + blueberry_cloud_shift_;
      new_x = face_center.x + 10;
    }

    else
      ROS_ERROR("[PCLUtilities] Invalid device type given: %s", device_.c_str());

    if ( new_y < 0 ) new_y = 0 + 10;
    if ( new_y > im_resolution_[1] ) new_y = im_resolution_[1] - 15 ;

    if ( new_x < 0 ) new_x = 0 + 10;
    if ( new_x > im_resolution_[0] ) new_x = im_resolution_[0] - 15;

    if ( new_x >= pt_cloud.width ){
      std::cout << "ERROR: new_x exceed pt_cloud size" << std::endl;
      std::cout << "new_x:  " << new_x << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    if ( new_y >= pt_cloud.height ){
      std::cout << "ERROR: new_y exceed pt_cloud size" << std::endl;
      std::cout << "new_y: " << new_y << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    coordinates[i].x = pt_cloud.at(new_x, new_y).x;
    coordinates[i].y = pt_cloud.at(new_x, new_y).y;
    coordinates[i].z = pt_cloud.at(new_x, new_y).z;

    if ( device_ == "Blueberry" )
      coordinates[i] = frameTransform ( coordinates[i], "kinect", "map");
  }

  return coordinates;
}

std::vector<cv::Point2f> PCLUtilities::getPointsToCheck(cv::Rect &potential){
  double percent_dist = 0.75; // What percentage of the way from potential.x to mid_point

  cv::Point mid_point ( potential.x + potential.width/2, potential.y + potential.height/2 );
  int radius = potential.width/2;

  int shift = potential.height;

  if ( potential.height * potential.width > 10000){
    ROS_ERROR("Huge box drawn");
    shift = 0;
    percent_dist = 0.85;
  }

  int x_min = potential.x + percent_dist*radius;
  int x_max = potential.x + potential.width - percent_dist*radius;

  int y_min = mid_point.y + shift - percent_dist*radius;
  int y_max = mid_point.y + shift + percent_dist*radius;

  //int x_to_check_tmp[] = { mid_point.x, x_min, x_min, x_max, x_max };
  //int y_to_check_tmp[] = { mid_point.y + shift, y_min, y_max, y_min, y_max };
  //std::vector<int> x_to_check = utils::createVector(x_to_check_tmp);
  //std::vector<int> y_to_check = utils::createVector(y_to_check_tmp);

  std::vector<int> x_to_check = { mid_point.x, x_min, x_min, x_max, x_max };
  std::vector<int> y_to_check = { mid_point.y + shift, y_min, y_max, y_min, y_max };

  //std::vector<int> x_to_check (x_to_check_tmp, x_to_check_tmp + sizeof(x_to_check_tmp) / sizeof (x_to_check_tmp[0]) );
  //std::vector<int> y_to_check (y_to_check_tmp, y_to_check_tmp + sizeof(y_to_check_tmp) / sizeof (y_to_check_tmp[0]) );

  std::vector<cv::Point2f> points_to_check(x_to_check.size());
  for (unsigned int i = 0; i < points_to_check.size(); i++)
    points_to_check[i] = cv::Point2f(x_to_check[i], y_to_check[i]);

  return points_to_check;
}

std::vector<cv::Point2f> PCLUtilities::getPointsToCheck2(cv::Rect &potential){
  int spacing = 3;

  std::vector<cv::Point2f> points_to_check(((potential.width/spacing)-1) * ((potential.height / spacing)-1));

  int indx = 0;
  for(int i = 1; i < potential.width/spacing; i++){
    for (int j = 1; j < potential.height/spacing; j++){
      int x = potential.x + (i * spacing);
      int y = potential.y + (j * spacing);
      
      points_to_check[indx] = cv::Point2f(x,y);
      indx++;
    }
  }

  return points_to_check;
}

cv::Point3f PCLUtilities::getCoordinate(pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Rect &rect){
  cv::Point2f center = utils::getCenter(rect);
  cv::Point3f coord = cv::Point3f(15,15,15);

  if (center.x < 0 || center.x > cloud.width)
    ROS_ERROR("[PCLUtilities] x coordinate exceeds cloud size: %d", (int)center.x);
  else if (center.y < 0 || center.y > cloud.height)
    ROS_ERROR("[PCLUtilities] y coordinate exceeds cloud size: %d", (int)center.y);
  else{
    coord.x = cloud.at(center.x, center.y).x;
    coord.y = cloud.at(center.x, center.y).y;
    coord.z = cloud.at(center.x, center.y).z;
  }

  return coord;
}

std::vector<cv::Point3f> PCLUtilities::getCoordinateinBaseFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect &potential){
  std::vector<cv::Point2f> points_to_check = getPointsToCheck(potential);
  std::vector<cv::Point3f> coordinates(points_to_check.size());
  
  for ( unsigned int i = 0; i < points_to_check.size(); i++ ){
    if ( points_to_check[i].x < 0 ) points_to_check[i].x = 0 + 10;
    if ( points_to_check[i].x > im_resolution_[0] ) points_to_check[i].x = im_resolution_[0] - 15;

    if ( points_to_check[i].y < 0 ) points_to_check[i].y = 0 + 10;
    if ( points_to_check[i].y > im_resolution_[1] ) points_to_check[i].y = im_resolution_[1] - 15 ;

    if ( points_to_check[i].x >= pt_cloud.width ){
      std::cout << "ERROR: x coordinate exceed pt_cloud size" << std::endl;
      std::cout << " x:  " << points_to_check[i].x << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    if ( points_to_check[i].y >= pt_cloud.width ){
      std::cout << "ERROR: y coordinate exceed pt_cloud size" << std::endl;
      std::cout << " y:  " << points_to_check[i].y << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    coordinates[i].x = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).x;
    coordinates[i].y = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).y;
    coordinates[i].z = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).z;

    if ( device_ == "Blueberry" )
      coordinates[i] = frameTransform ( coordinates[i], "kinect", "base");
  }

  return coordinates;
}

bool PCLUtilities::isGoodCoordinate(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Point3f &coordinate, cv::Rect potential, double min, double max){
  std::vector<cv::Point3f> pts = getCoordinateinBaseFrame(pt_cloud, potential);

  double tol = 0.25;

  coordinate = cv::Point3f(0,0,0);

  double min_value = 16;
  double max_value = -16;
  
  for (unsigned int i = 0; i < pts.size(); i++){
    if (std::isnan(pts[i].x)){
      //ROS_ERROR("Point Cloud returned NAN");
      return false;
    }

    if ( ( pts[i].x >= max ) || ( pts[i].x <= min ) ){
      //ROS_ERROR("Point out of given range");
      return false;
    }

    coordinate += pts[i];

    if ( pts[i].x < min_value && device_ == "Blueberry")
      min_value = pts[i].x;
    if ( pts[i].x > max_value && device_ == "Blueberry")
      max_value = pts[i].x;
    if ( pts[i].z < min_value && device_ == "Kinect")
      min_value = pts[i].z;
    if ( pts[i].z > max_value && device_ == "Kinect")
      max_value = pts[i].z;
  }

  if ( fabs(max_value-min_value) > tol ){
    //ROS_ERROR("Points to far apart");
    return false;      
  }

  coordinate.x /= 5;
  coordinate.y /= 5;
  coordinate.z /= 5;

  return true;
}

std::vector<cv::Point3f> PCLUtilities::getCoordinateinBaseFrame (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect &potential, std::vector<cv::Point2f> &points_to_check){
  points_to_check = getPointsToCheck(potential);
  
  return getBatchCoordinates(pt_cloud, points_to_check);
}

bool PCLUtilities::isGoodCoordinate(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<bool> &is_good_coord, std::vector<cv::Point2f> &im_coord, std::vector<cv::Point3f> &coord, cv::Point3f &avg_coord, cv::Rect potential, double min, double max){
  coord = getCoordinateinBaseFrame(pt_cloud, potential, im_coord);

  double tol = 0.25;

  avg_coord = cv::Point3f(0,0,0);

  double min_value = 16;
  double max_value = -16;

  is_good_coord.resize(coord.size());

  bool good = true;
  
  for (unsigned int i = 0; i < coord.size(); i++){
    bool is_nan = std::isnan(coord[i].x);
    bool is_outside_range = ( ( coord[i].x >= max ) || ( coord[i].x <= min ) );

    if (is_nan || is_outside_range )
      is_good_coord[i] = false;

    else
      is_good_coord[i] = true;

    if (is_nan)
      good = false;
      
    avg_coord += coord[i];

    if ( coord[i].x < min_value && device_ == "Blueberry")
      min_value = coord[i].x;
    if ( coord[i].x > max_value && device_ == "Blueberry")
      max_value = coord[i].x;
    if ( coord[i].z < min_value && device_ == "Kinect")
      min_value = coord[i].z;
    if ( coord[i].z > max_value && device_ == "Kinect")
      max_value = coord[i].z;
  }

  avg_coord.x /= 5;
  avg_coord.y /= 5;
  avg_coord.z /= 5;

  if ( fabs(max_value-min_value) > tol )
    good = false;

  return good;
}


cv::Point3f PCLUtilities::frameTransform ( cv::Point3f current_frame_coordinates, std::string frameFrom, std::string frameTo ){
	cv::Point3f new_frame_coordinates;

  if (frameTo == frameFrom || device_ != "Blueberry" )
    return current_frame_coordinates;

	if ( frameTo == "kinect" ) frameTo = kinect_frame_;
	else if ( frameTo == "map" ) frameTo = map_frame_;
	else if ( frameTo == "base" ) frameTo = base_frame_;
  else 
    ROS_ERROR("[PCLUtilities] Invalid frameTo: %s", frameTo.c_str());

	if ( frameFrom == "kinect" ) frameFrom = kinect_frame_;
	else if ( frameFrom == "map" ) frameFrom = map_frame_;
	else if ( frameFrom == "base" ) frameFrom = base_frame_;
  else 
    ROS_ERROR("[PCLUtilities] Invalid frameFrom: %s", frameFrom.c_str());

	if ( device_ == "Blueberry" ){
		geometry_msgs::PoseStamped currentPose = createPose ( frameFrom, current_frame_coordinates.x, current_frame_coordinates.y, current_frame_coordinates.z, 0, 0, 0);
		geometry_msgs::PoseStamped newPose = transformPose ( frameTo, currentPose );
  	new_frame_coordinates.x = newPose.pose.position.x;
  	new_frame_coordinates.y = newPose.pose.position.y;
		new_frame_coordinates.z = newPose.pose.position.z + head_offset_;
		//std::cout << "Converting from " << frameFrom << " to " << frameTo << std::endl;
		//std::cout << "  " << frameFrom << " Coordinates: " << current_frame_coordinates.x << ", " << current_frame_coordinates.y << ", " << current_frame_coordinates.z << std::endl;
		//std::cout << "  " << frameTo << " Coordinates: " <<   new_frame_coordinates.x << ", " << new_frame_coordinates.y << ", " << new_frame_coordinates.z << std::endl;
	}

	else
		new_frame_coordinates = current_frame_coordinates;

	return new_frame_coordinates;
}

void PCLUtilities::getMinMaxPCL (pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Point3f &min, cv::Point3f &max){
  pcl::PointXYZ pcl_min, pcl_max;
  pcl::getMinMax3D(pt_cloud, pcl_min, pcl_max);

  min.x = pcl_min.x;   min.y = pcl_min.y;   min.z = pcl_min.z;
  min.y = pcl_max.x;   max.y = pcl_max.y;   max.z = pcl_max.z;

  return;
}

Rpose PCLUtilities::getRobotPose(std::string frame){
  Rpose R;

  if ( frame == "kinect" ) frame = kinect_frame_;
  else if ( frame == "map" ) frame = map_frame_;
  else if ( frame == "base" ) frame = base_frame_;
  else 
    ROS_ERROR("[PCLUtilities] Invalid frame: %s", frame.c_str());

  geometry_msgs::PoseStamped robot_pose = createPose(base_frame_, 0, 0, 0, 0, 0, 0);
  robot_pose = transformPose(frame, robot_pose);

  tf::Pose pose;
  tf::poseMsgToTF(robot_pose.pose, pose);

  R.x = robot_pose.pose.position.x;
  R.y = robot_pose.pose.position.y;
  R.th = utils::modTo180(utils::radToDeg(tf::getYaw(pose.getRotation())));

  return R;
}

cv::Point3f PCLUtilities::getAvgCoord(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, cv::Rect& rect){
  int min_x = rect.x + rect.width/3;
  int min_y = rect.y + rect.height/3;
  int max_x = rect.x + rect.width - rect.width/3;
  int max_y = rect.y + rect.height - rect.height/3;

  float x_spacing = float(max_x - min_x)/float(avg_coord_granularity_+1);
  float y_spacing = float(max_y - min_y)/float(avg_coord_granularity_+1);  

  std::vector<cv::Point2f> points_to_check(pow(avg_coord_granularity_,2));
  int indx = 0;

  for(int i = 1; i < avg_coord_granularity_+1; i++){
    for(int j = 1; j < avg_coord_granularity_+1; j++){
      points_to_check[indx] = cv::Point2f(
        int(min_x + x_spacing*i),
        int(min_y + y_spacing*j)
      );
      indx++;
    } 
  }

  std::vector<cv::Point3f> coords = getBatchCoordinates(pt_cloud, points_to_check);
  double valid_pts = 0;
  cv::Point3f avg_coord(0,0,0);

  for(unsigned int i = 0; i < coords.size(); i++){
    if(coords[i].x != 15){
      avg_coord += coords[i];
      //std::cout << coords[i] << std::endl;
      valid_pts++;
    }
  }

  if(valid_pts != 0)
    avg_coord /= valid_pts;

  return avg_coord;
}

geometry_msgs::PoseStamped PCLUtilities::createPose(std::string frame, double x, double y, double z, double ax, double ay, double az) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = ax;
  pose.pose.orientation.y = ay;
  pose.pose.orientation.z = az;
  pose.pose.orientation.w = 1.0;
  pose.header.stamp = ros::Time::now();
  return pose;
}

geometry_msgs::PoseStamped PCLUtilities::transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom){
  geometry_msgs::PoseStamped poseTo;

  try{
    ros::Time current_time = ros::Time::now();
    poseFrom.header.stamp = current_time;
    poseTo.header.stamp = current_time;
    tfListener_.waitForTransform(frameTo, poseFrom.header.frame_id, current_time, ros::Duration(1.0));
    tfListener_.transformPose(frameTo, poseFrom, poseTo);
  } 

  catch (tf::TransformException ex){
    std::cout << ex.what() << std::endl;
  }

  return poseTo;
}

std::vector<cv::Point3f> PCLUtilities::getBatchCoordinates(pcl::PointCloud<pcl::PointXYZ> &pt_cloud, std::vector<cv::Point2f> &points_to_check){
  std::vector<cv::Point3f> coordinates(points_to_check.size()); 
  for ( unsigned int i = 0; i < points_to_check.size(); i++ ){
    if ( points_to_check[i].x < 0 ) points_to_check[i].x = 0 + 10;
    if ( points_to_check[i].x > im_resolution_[0] ) points_to_check[i].x = im_resolution_[0] - 15;

    if ( points_to_check[i].y < 0 ) points_to_check[i].y = 0 + 10;
    if ( points_to_check[i].y > im_resolution_[1] ) points_to_check[i].y = im_resolution_[1] - 15 ;

    if ( points_to_check[i].x >= pt_cloud.width ){
      std::cout << "ERROR: x coordinate exceed pt_cloud size" << std::endl;
      std::cout << " x:  " << points_to_check[i].x << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    if ( points_to_check[i].y >= pt_cloud.width ){
      std::cout << "ERROR: y coordinate exceed pt_cloud size" << std::endl;
      std::cout << " y:  " << points_to_check[i].y << std::endl;
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue;
    }

    coordinates[i].x = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).x;
    coordinates[i].y = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).y;
    coordinates[i].z = pt_cloud.at(points_to_check[i].x, points_to_check[i].y).z;

    if ( std::isnan(coordinates[i].x) ){
      coordinates[i].x = 15;
      coordinates[i].y = 15;
      coordinates[i].z = 15;
      continue; 
    }    

    if ( device_ == "Blueberry" )
      coordinates[i] = frameTransform ( coordinates[i], "kinect", "base");
  }

  return coordinates;
}

void PCLUtilities::loadROSParams(std::string ns){
 
  nh_.param(ns + "/device", device_, device_);

  // Image Params
  nh_.param(ns + "/image" + "/used_resolution", im_resolution_, im_resolution_);

  // Frame Params
  nh_.param(ns + "/frames" + "/kinect", kinect_frame_, kinect_frame_);
  nh_.param(ns + "/frames" + "/robot", base_frame_, base_frame_);
  nh_.param(ns + "/frames" + "/map", map_frame_, map_frame_);

  // PCLUtilities Param
  nh_.param(ns + "/PCLUtilities" + "/head_offset", head_offset_, head_offset_);
  nh_.param(ns + "/PCLUtilities" + "/kinect_cloud_shift", kinect_cloud_shift_, kinect_cloud_shift_);
  nh_.param(ns + "/PCLUtilities" + "/blueberry_cloud_shift", blueberry_cloud_shift_, blueberry_cloud_shift_);
  nh_.param(ns + "/PCLUtilities" + "/avg_coord_granularity", avg_coord_granularity_, avg_coord_granularity_);

  if (nh_.hasParam(ns + "/tf_prefix")) {
    std::string tf_prefix = "";
    nh_.param(ns + "/tf_prefix", tf_prefix, tf_prefix);
    if(device_ == "Kinect" || device_ == "Xtion")
      kinect_frame_ = tf_prefix + "_" + kinect_frame_;
    else
      kinect_frame_ = tf_prefix + "/" + kinect_frame_;
    base_frame_ = tf_prefix + "/" + base_frame_;
  }
  
  if ( !(device_ == "Kinect" || device_ == "Blueberry" || device_ == "Realsense") )
    ROS_ERROR("[PCLUtilities] Invalid device: %s", device_.c_str());

  ROS_INFO("[PCLUtilities] ROS Params Updated");
}

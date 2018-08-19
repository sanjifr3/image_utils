#include <ros/ros.h>
#include <jetson_visualizer/draw_rviz.h>
#include <string>
#include <std_msgs/Bool.h>

// Occupancy Grid Type
#include <visualization_msgs/Marker.h>

int drawn_shape_ctr = 0;

ros::Publisher markerPub;

visualization_msgs::Marker marker;
uint32_t cube = visualization_msgs::Marker::CUBE;
uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
uint32_t arrow = visualization_msgs::Marker::ARROW;

void setColor (std::string color);
bool drawMarker(jetson_visualizer::draw_rviz::Request  &req,
                jetson_visualizer::draw_rviz::Response &res);
bool drawBox(jetson_visualizer::draw_rviz::Request  &req,
             jetson_visualizer::draw_rviz::Response &res);
bool deleteAllMarkers(jetson_visualizer::draw_rviz::Request  &req,
                      jetson_visualizer::draw_rviz::Response &res);
bool drawRVizCb(jetson_visualizer::draw_rviz::Request  &req,
                jetson_visualizer::draw_rviz::Response &res);


int main (int argc, char** argv)
{
  ros::init(argc, argv, "rviz_drawing_server");
  ros::NodeHandle nh;

  markerPub = nh.advertise<visualization_msgs::Marker>("jetson_visualizer/markers", 1);

  marker.ns = "LocalRPS";
  marker.header.frame_id = "map";
  marker.lifetime = ros::Duration();

  ros::ServiceServer service = nh.advertiseService("DrawRVIZ", drawRVizCb);
  ROS_INFO("RViz Drawing Server Enabled");

  ros::spin();
  return 0;
}

void setColor (std::string color)
{
  marker.color.b = 0.0f;
  marker.color.g = 0.0f;
  marker.color.r = 0.0f;

  if ( color == "red" )
    marker.color.r = 1.0f;

  else if ( color == "blue" )
    marker.color.b = 1.0f;

  else if ( color == "green" )
    marker.color.g = 1.0f;

  else if ( color == "light blue" )
    marker.color.b = 0.5f;

  else if ( color == "light red" )
    marker.color.r = 0.5f;

  else if ( color == "light green" )
    marker.color.g = 0.5f;

  else if ( color == "black" )
  {
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }

  else if ( color == "gray" )
  {
    marker.color.b = 0.5f;
    marker.color.g = 0.5f;
    marker.color.r = 0.5f;
  }

  else if ( color == "white" )
  {
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
  }

  else if ( color == "orange" )
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.65f;
  }

  else if ( color== "dark gray" )
  {
    marker.color.b = 0.17f;
    marker.color.g = 0.17f;
    marker.color.r = 0.17f;
  }

  else if ( color== "yellow" )
  {
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
  }

  else if ( color== "cyan" )
  {
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
  }

  else if ( color == "magenta" )
  {
    marker.color.r = 1.0f;
    marker.color.b = 1.0f;
  }
}

bool drawMarker(jetson_visualizer::draw_rviz::Request  &req,
                jetson_visualizer::draw_rviz::Response &res)
{
  // Figure out angles for arrow
  double heading = 0; 
  double altitude = req.angle;
  if ( altitude == 180 || altitude == -180 )
    altitude = 179;

  double bank = 0;

  heading = heading*M_PI/180;
  altitude = altitude*M_PI/180;
  bank = bank*M_PI/180;

  double c1 = cos(heading/2);
  double c2 = cos(altitude/2);
  double c3 = cos(bank/2);
  double s1 = sin(heading/2);
  double s2 = sin(altitude/2);
  double s3 = sin(bank/2);

  // Mutual parameters
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.a = req.alpha;
  setColor(req.color);
  marker.pose.position.x = req.coordinate.x;
  marker.pose.position.y = req.coordinate.y;
  marker.pose.position.z = req.coordinate.z/2;
  marker.scale.y = 0.20;

  // Draw cylinder
  marker.id = drawn_shape_ctr;
  marker.type = cylinder;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.15;
  marker.scale.z = req.coordinate.z;

  marker.header.stamp = ros::Time::now();
  markerPub.publish(marker);
  drawn_shape_ctr++;

  // Draw arrow
  marker.id = drawn_shape_ctr;
  marker.type = arrow;
  marker.pose.orientation.x = s1*s2*c3 + c1*c2*s3;
  marker.pose.orientation.y = s1*c2*c3 + c1*s2*s3;
  marker.pose.orientation.z = c1*s2*c3 - s1*c2*s3;
  marker.pose.orientation.w = c1*c2*c3 - s1*s2*s3;
  marker.scale.x = 0.7;
  marker.scale.z = 0.3;

  marker.header.stamp = ros::Time::now();
  markerPub.publish(marker);
  drawn_shape_ctr++;

  res.success = true;

  return true;
}

bool drawBox(jetson_visualizer::draw_rviz::Request  &req,
             jetson_visualizer::draw_rviz::Response &res)
{
  marker.id = drawn_shape_ctr;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = cube;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 2.0;
  marker.scale.y = 2.0;
  marker.scale.z = 0.25;

  setColor(req.color);
  marker.color.a = req.alpha;

  marker.pose.position.x = req.coordinate.x;
  marker.pose.position.y = req.coordinate.y;

  marker.header.stamp = ros::Time::now();
  markerPub.publish(marker);
  drawn_shape_ctr++;

  res.success = true;

  return true;
}

bool deleteAllMarkers(jetson_visualizer::draw_rviz::Request  &req,
                      jetson_visualizer::draw_rviz::Response &res)
{
  marker.action = 3;

  for (int i = 0; i < drawn_shape_ctr; i++)
  {
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    markerPub.publish(marker);
  }

  drawn_shape_ctr = 0;

  res.success = true;

  return true;
}

bool drawRVizCb(jetson_visualizer::draw_rviz::Request  &req,
                jetson_visualizer::draw_rviz::Response &res)
{
  bool status = true;
  if(req.type == "box")
    status = drawBox(req, res);

  else if (req.type == "marker")
    status = drawMarker(req, res);

  else if (req.type == "delete")
    status = deleteAllMarkers(req, res);

  else
  {
    status = false;
    std::cout << "RVIZDrwaingServer: Invalid type sent to RViz drawing server" << std::endl;
  }

  return status;
}

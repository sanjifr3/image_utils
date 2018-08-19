#include <social_robot/Utilities.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "makePlan_client");
  ros::NodeHandle nh;

  ros::ServiceClient mp_client = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
  ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("plan",1);

  nav_msgs::GetPlan msg;

  msg.request.start.header.frame_id = "map";
  msg.request.goal.header.frame_id = "map";
  msg.request.tolerance = 0.0;

  while(ros::ok()){
    double x, y, start_th, end_th;
    std::cout << "Enter robot current position: ";
    std::cin >> x >> y >> start_th;
    msg.request.start.pose.position.x = x;
    msg.request.start.pose.position.y = y;
    msg.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(start_th*M_PI/180.0);

    std::cout << "Enter robot final position: ";
    std::cin >> x >> y >> end_th;
    msg.request.goal.pose.position.x = x;
    msg.request.goal.pose.position.y = y;
    msg.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(end_th*M_PI/180.0);

    if (mp_client.call(msg)){
      msg.response.plan.header.frame_id = "map";
      plan_pub.publish(msg.response.plan);

      std::vector<geometry_msgs::PoseStamped> &pts = msg.response.plan.poses;
      double euc_dist = 0;

      for(int i = 0; i < (int) pts.size()-1; i++){
        cv::Point2f p1 = cv::Point2f(pts[i].pose.position.x, pts[i].pose.position.y);
        cv::Point2f p2 = cv::Point2f(pts[i+1].pose.position.x, pts[i+1].pose.position.y);
        euc_dist += sr_utils::euclideanDistance(p1,p2);
      }

      double ang_dist = fabs(end_th - start_th);

      std::cout << "waypoints: " << pts.size() << std::endl;
      std::cout << "linear distance: " << euc_dist << std::endl;
      std::cout << "angular distance: " << ang_dist << std::endl;
    }

    else
      ROS_ERROR("Failed to call service move_base/NavfnROS/make_plan");
  }  


  return 0;
}
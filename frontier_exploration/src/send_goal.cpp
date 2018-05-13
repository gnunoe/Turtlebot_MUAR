#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_exploration_goal");

  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> client("explore_server", true);

  ROS_INFO("Waiting for action server");//
  client.waitForServer();//
  ROS_INFO("Sending goal");//
  frontier_exploration::ExploreTaskGoal goal;//

  geometry_msgs::PolygonStamped boundary;
  geometry_msgs::Point32 p1, p2, p3, p4, p5;

  float dim = 18.5;

  p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
  p2.x = -dim; p2.y = 0.0; p2.z = 0.0;
  p3.x = -dim; p3.y = dim; p3.z = 0.0;
  p4.x = 0.0; p4.y = dim; p4.z = 0.0;
  p5.x = p1.x; p5.y = p1.y; p5.z = p1.z;

  boundary.header.seq = 0;
  boundary.header.stamp = ros::Time::now();
  boundary.header.frame_id = "map";

  boundary.polygon.points.push_back(p1);
  boundary.polygon.points.push_back(p2);
  boundary.polygon.points.push_back(p3);
  boundary.polygon.points.push_back(p4);
  boundary.polygon.points.push_back(p5);

  goal.explore_center.header.frame_id = "map";
  goal.explore_center.point.x = -8.5;//
  goal.explore_center.point.y = 8.5; //
  goal.explore_center.point.z = 0.0; //
  goal.explore_boundary = boundary;
  //cout << boundary.polygon << endl;
  //cout << boundary << endl;

  client.sendGoal(goal);

}

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include <sstream>


bool aruco_found = false;
bool aruco_close = false;
int desired_object = 0;
int obj_pos [12][4]; //Save position of desired object

//Aruco id callback
void aruco_id_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
  if (aruco_found == false)
  { 
    ROS_INFO("Aruco marker %i found", msg->data);
    desired_object = msg->data;
    aruco_found = true;
  }
}

//Aruco pose callback
void aruco_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (msg->position.z < 2.0)
  {
    ROS_INFO("Aruco marker at %f meters.", msg->position.z);
    aruco_close = true;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "scheduler");
  ros::NodeHandle n;
  
  //Aruco publishers and subscriptors
  ros::Subscriber aruco_id_sub = n.subscribe("aruco_simple/arucoID", 1, aruco_id_Callback);
  ros::Subscriber aruco_pose_sub = n.subscribe("aruco_simple/pose", 1, aruco_pose_Callback);
  ros::Publisher cancel_nav_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  actionlib_msgs::GoalID stop_nav;
  
  ROS_INFO("Stage 1: Searching for the Aruco Marker...");
  //start navigation 
  system("rosrun simple_navigation_goals simple_navigation_goals -17 17");

  ros::Rate loop_rate(10);
  int counter = 0;
  int foo_obj_found = 1;

  while(ros::ok())
  {
    while ((aruco_found == false || aruco_close == false) && ros::ok())
    {
      //ROS_INFO(".");
    
      //ToDo
      //If any object is detected
      /*if (counter % 50 == 0)
      { 
        ROS_INFO("Fake Object Found!");
        std::ostringstream num;
        num << foo_obj_found;
        std::string cmd = "rosrun simple_navigation_goals object_mapping_request " + num.str();
        system(cmd.c_str());
        foo_obj_found++;
      }*/

      //If a coin is detected
        //bla bla

      ++counter;
      ros::spinOnce();
      loop_rate.sleep();

      //if (!ros::ok())
      //{
      //  ros::shutdown();
      //} 
    }

  
    ROS_INFO("Canceling Aruco Node and Navigation...");
    system("rosnode kill aruco_simple");
    cancel_nav_pub.publish(stop_nav);
    aruco_id_sub.shutdown();
    aruco_pose_sub.shutdown();  

    n.setParam("/Object_ID", desired_object);


    ROS_INFO("Stage 2: Exploring the area to search for desired object...");
    system("rosrun frontier_exploration unbound_exploration");
  
  }
  return 0;
}

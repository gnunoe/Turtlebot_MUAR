#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include <sstream>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "simple_navigation_goals/getWorldPose.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

class scheduler
{
public:
  
  //0 = Searching for Aruco
  //1 = Exploring
  //2 = Rescuing object
  //3 = Returning for Aruco
  int stage;

  bool aruco_found;
  bool aruco_close;
  bool aruco_update_pose;
  float aruco_pose[2];
  int desired_object;
  //Matrix with following fields
  // 1. Object Already Detected (0=no, 1=yes)
  // 2. Full ID which represents the object (1-->12)
  // 3. Probability of detection (0-->1)
  // 4,5,6. X,Y and Z world coordinates
  float obj_pos [12][6]; //Save position of desired object

  int coin_counter;
  bool heading_coin;
  bool hit_coin;
  float dist_thr;
  float dist_update;
  float coin_pos[4]; //Coin probability and coordinates
  float coin_array[11][4];
  int coin_issues;

  ros::ServiceClient tf_map_cam_client;
  ros::Publisher cancel_nav_pub;
  ros::Publisher cancel_explore_pub;
  actionlib_msgs::GoalID stop_nav;
  int current_nav_status;
  int current_explore_status;

  void aruco_id_Callback(const std_msgs::UInt8::ConstPtr& msg);
  void aruco_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg);
  void yolo_obj_found_Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  int get_object_ID(std::string obj_name);
  void spawn_RVIZ_object(int object_ID, float x, float y, float z);
  void send_nav_goal(float x, float y);
  void coin_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void navStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
  void exploreStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
  void recover_status();
  void check_coin_issue();
  void robot_aruco_close();

  scheduler()
  {
    stage = 0;
    
    aruco_found = false;
    aruco_close = false;
    aruco_update_pose = false;
    aruco_pose[0] = -17;
    aruco_pose[1] = 17;
    desired_object = 0;

    for (int i=0; i<12; i++)
    {
      for (int j=0; j<6; j++)
      {
        obj_pos[i][j]=0.0;
      }
    }
    
    coin_counter = 0;
    heading_coin = false;
    hit_coin = false;
    dist_thr = 0.7;
    dist_update = 1.0;
    for (int i=0; i<4; i++)
    {
      coin_pos[i]=0.0;
    }
    for (int i=0; i<10; i++)
    {
      for (int j=0; j<4; j++)
      {
        coin_array[i][j]=0.0;
      }
    }

    coin_issues = 0;
    current_nav_status = 99;
    current_explore_status = 99;
  };


  ~scheduler()
  {

  };

private:

};


//If robot is in the final goal but not facing aruco
void scheduler::robot_aruco_close()
{
  simple_navigation_goals::getWorldPose srv;
  srv.request.cam_x = 0;
  srv.request.cam_y = 0;
  srv.request.cam_z = 0;
  tf_map_cam_client.call(srv);
  float rob_x = srv.response.world_x;
  float rob_y = srv.response.world_y;
  float rob_z = srv.response.world_z; 
  
  float dx = abs(-17 -rob_x);
  float dy = abs( 17 -rob_y);
  float thr = 0.7;
  if (dx <= thr && dy <= thr && aruco_found == true)
  {
    aruco_close = true;
  }
}

//Coin problems, forget about coin and jkeep up with mision
void scheduler::check_coin_issue()
{
  if ((heading_coin == true && current_nav_status == 3 && coin_issues >= 5 && stage == 0) ||
      (heading_coin == true && current_explore_status ==3 && coin_issues >= 5 && stage == 1))
  {
    heading_coin = false;
    coin_issues = 0;
    ROS_INFO("Map drifts made impossible to get coin, continuing with mission");
    recover_status();
  }
  coin_issues = coin_issues + 1;
} 

//Recover status
void scheduler::recover_status()
{
  if (stage == 0)
  {
    cancel_nav_pub.publish(stop_nav);
    ros::Duration(1).sleep();
    ROS_INFO("Continuing with the Aruco Search");
    send_nav_goal(aruco_pose[0], aruco_pose[1]);
  }
  else if (stage == 1)
  {
    cancel_explore_pub.publish(stop_nav);
    ros::Duration(1).sleep();
    ROS_INFO("Continuing with the Exploration");
    system("rosrun frontier_exploration unbound_exploration");
  }
}

//Explore Status Callback
void scheduler::exploreStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  if (!msg->status_list.empty())
  {
    int last_id = msg->status_list.size()-1;
    current_explore_status = msg->status_list[last_id].status;
  }
}

//Nav Status Callback
void scheduler::navStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  //nav_status_array = *msg;
  if (!msg->status_list.empty())
  {
    int last_id = msg->status_list.size()-1;
    current_nav_status = msg->status_list[last_id].status;
    //ROS_INFO("--------%i--------",int(msg->status_list.size()));
    /*ROS_INFO("--------%i--------",int(msg->status_list.size()));
    for (int i= 0; i<msg->status_list.size(); i++)
    {
      ROS_INFO("%i", msg->status_list[i].status);
    }*/
  }
}

//Coin callback
void scheduler::coin_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  //If we really were heading a coin and we hitted it
  if (msg->states.size() > 0) //heading_coin == true && 
  {
    heading_coin = false;
    coin_issues = 0;
    coin_counter = coin_counter + 1;
    ROS_INFO("Coin n %i catched!",coin_counter);
    recover_status();
  }
}

//Aruco id callback
void scheduler::aruco_id_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
  if (aruco_found == false)
  { 
    ROS_INFO("Aruco marker %i found", msg->data);
    desired_object = msg->data;
    aruco_found = true;
    aruco_update_pose = true;
  }
}

//Aruco pose callback
void scheduler::aruco_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (aruco_update_pose == true && msg->position.z < 3.0) 
  {
    aruco_update_pose = false;
    ROS_INFO("Aruco found at [%f, %f, %f]", 
             msg->position.x, msg->position.y, msg->position.z); 
  } 

  if (msg->position.z < 2.0)
  {
    ROS_INFO("Aruco marker at %f meters.", msg->position.z);
    aruco_close = true;
  }
}



//Yolo Object Found callback
void scheduler::yolo_obj_found_Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  //ROS_INFO("%i objects found", num_obj_found);
  int local_id = 0;
  int arr_ind = 0;
  for (int i = 0; i < msg->bounding_boxes.size(); i++) 
  {
    //object found
    if (msg->bounding_boxes[i].Class != "moneda" && msg->bounding_boxes[i].Class != "bomba")
    {
      //Only if we can estimate the 3D coordinates (!=NaN) we add/update 3D coordinates
      //and the probability is greater than 60%
      if (!isnan(msg->bounding_boxes[i].X) && !isnan(msg->bounding_boxes[i].Y) 
          && !isnan(msg->bounding_boxes[i].Z) && msg->bounding_boxes[i].probability >=0.3)
      {
        local_id = get_object_ID(msg->bounding_boxes[i].Class);
        arr_ind = local_id - 1;
        switch((int)obj_pos[arr_ind][0])
        {
          case 0: //First entry of the object
          {
            ROS_INFO("First view of the object %i, saving coordinates...", local_id );
            obj_pos[arr_ind][0]=1;
            obj_pos[arr_ind][1]=local_id;
            obj_pos[arr_ind][2]=msg->bounding_boxes[i].probability;
            simple_navigation_goals::getWorldPose srv;
            srv.request.cam_x = msg->bounding_boxes[i].X;
            srv.request.cam_y = msg->bounding_boxes[i].Y;
            srv.request.cam_z = msg->bounding_boxes[i].Z;
            tf_map_cam_client.call(srv);
            obj_pos[arr_ind][3] = srv.response.world_x;
            obj_pos[arr_ind][4] = srv.response.world_y;
	    obj_pos[arr_ind][5] = srv.response.world_z;
            spawn_RVIZ_object(obj_pos[arr_ind][1], obj_pos[arr_ind][3], 
                              obj_pos[arr_ind][4], obj_pos[arr_ind][5]);
            break;
          }
          case 1: //Object already detected
          { 
            ROS_INFO("Object %i already detected, checking previous estimation...", local_id);
	    if (obj_pos[arr_ind][2] > msg->bounding_boxes[i].probability)
            {
              ROS_INFO("Worse estimation, skipping coordinates update...");
              break;
            }
            else
            {
              ROS_INFO("Better estimation, updating coordinates...");
              obj_pos[arr_ind][2]=msg->bounding_boxes[i].probability;
              simple_navigation_goals::getWorldPose srv;
              srv.request.cam_x = msg->bounding_boxes[i].X;
              srv.request.cam_y = msg->bounding_boxes[i].Y;
              srv.request.cam_z = msg->bounding_boxes[i].Z;
              tf_map_cam_client.call(srv);
              obj_pos[arr_ind][3] = srv.response.world_x;
              obj_pos[arr_ind][4] = srv.response.world_y;
	      obj_pos[arr_ind][5] = srv.response.world_z;
              spawn_RVIZ_object(obj_pos[arr_ind][1], obj_pos[arr_ind][3], 
                                obj_pos[arr_ind][4], obj_pos[arr_ind][5]);
              break;
            }
          }
          default:
          {
 	    break;
          }
        } //switch
      } //if not NaN
      
    } //if object found
    //if moneda found
    else if (msg->bounding_boxes[i].Class == "moneda")
    {
      //ROS_INFO("Moneda");
      //Only if we can estimate the 3D coordinates (!=NaN) we add/update 3D coordinates
      if (!isnan(msg->bounding_boxes[i].X) && !isnan(msg->bounding_boxes[i].Y) 
          && !isnan(msg->bounding_boxes[i].Z))
      {
        //As it's computationally, inexpensive, first we get the position of the coin
        simple_navigation_goals::getWorldPose srv;
        srv.request.cam_x = msg->bounding_boxes[i].X;
        srv.request.cam_y = msg->bounding_boxes[i].Y;
        srv.request.cam_z = msg->bounding_boxes[i].Z;
        tf_map_cam_client.call(srv);
        float c_x = srv.response.world_x;
        float c_y = srv.response.world_y;	
        float c_z = srv.response.world_z;

        //And the position of the robot
        srv.request.cam_x = 0;
        srv.request.cam_y = 0;
        srv.request.cam_z = 0;
        tf_map_cam_client.call(srv);
        float rob_x = srv.response.world_x;
 	float rob_y = srv.response.world_y;
        float rob_z = srv.response.world_z;
        //ROS_INFO("Robot in [%f, %f, %f]",rob_x,rob_y,rob_z);

        //And the distance between the coin and the robot
        float dist_x = msg->bounding_boxes[i].X;
        float dist_y = msg->bounding_boxes[i].Y; 
        float dist_z = msg->bounding_boxes[i].Z;
        float dist_t = sqrt(dist_x*dist_x + dist_z*dist_z);

        //If we are not navigating against a coin and the coin is close
        if (heading_coin == false)
        {
          if (dist_t <= 3.0)
          {
            ROS_INFO("Near coin found! Heading against it to win some points!");
            heading_coin = true;
            coin_issues = 0;
            dist_update = 1.0;
            dist_thr = 0.7;

            ROS_INFO("KINECT: Coin in [%f, %f, %f]",c_x,c_y,c_z);
            coin_pos[0] = msg->bounding_boxes[i].probability;
            coin_pos[1] = c_x;
            coin_pos[2] = c_y;
            coin_pos[3] = c_z;

            int x_trend = -1; int y_trend = -1;
            if (c_x - rob_x > 0){x_trend = 1;}
  	    if (c_y - rob_y > 0){y_trend = 1;}
            float margin = 0.5;
 
            if (stage == 0){cancel_nav_pub.publish(stop_nav);}
            if (stage == 1){cancel_explore_pub.publish(stop_nav);}
            ros::Duration(1).sleep();
            send_nav_goal(c_x + margin*x_trend,
                          c_y + margin*y_trend);
            
          }
        }
        else //we are already navigating against a coin
        {
          if (dist_t <= dist_update) //It should be closer at this point
          {
            dist_update = dist_update - 0.5; //Only two updates possibles 

            ROS_INFO("Same coin Founded! Updated position from [%f,%f] to [%f,%f]",
                       coin_pos[1],coin_pos[2],c_x,c_y);

            coin_pos[0] = msg->bounding_boxes[i].probability;
            coin_pos[1] = c_x;
            coin_pos[2] = c_y;
            coin_pos[3] = c_z;
           
            int x_trend = -1; int y_trend = -1;
            if (c_x - rob_x > 0) {x_trend = 1;}
  	    if (c_y - rob_y > 0){y_trend = 1;}
            float margin = 0.5;
 
            if (stage == 0){cancel_nav_pub.publish(stop_nav);}
            if (stage == 1){cancel_explore_pub.publish(stop_nav);}
            ros::Duration(1).sleep();
            send_nav_goal(c_x + margin*x_trend,
                          c_y + margin*y_trend);

          }//dist_t<=dist_update 
        }//else already navigating
      } //else moneda
    }//if isnan
  } //for
} //void

int scheduler::get_object_ID(std::string obj_name)
{
  //Get object id
  if (obj_name == "avion azul"){return 1;}
  else if (obj_name == "avion verde"){return 2;}
  else if (obj_name == "avion rojo"){return 3;}
  else if (obj_name == "bicicleta azul"){return 4;}
  else if (obj_name == "bicicleta verde"){return 5;}
  else if (obj_name == "bicicleta roja"){return 6;}
  else if (obj_name == "barco azul"){return 7;}
  else if (obj_name == "barco verde"){return 8;}
  else if (obj_name == "barco rojo"){return 9;}
  else if (obj_name == "botella azul"){return 10;}
  else if (obj_name == "botella verde"){return 11;}
  else if (obj_name == "botella roja"){return 12;}
}

void scheduler::spawn_RVIZ_object(int object_ID, float x, float y, float z)
{
  std::ostringstream arg1;
  std::ostringstream arg2;
  std::ostringstream arg3;
  std::ostringstream arg4;
  arg1 << object_ID;
  arg2 << x;
  arg3 << y;
  arg4 << z;
  std::string cmd = "rosrun simple_navigation_goals object_mapping_request " + arg1.str() 
                     + " " + arg2.str() + " " + arg3.str() + " " + arg4.str(); 
  system(cmd.c_str());
}

void scheduler::send_nav_goal(float x, float y)
{
  std::ostringstream arg1;
  std::ostringstream arg2;
  arg1 << x;
  arg2 << y;
  std::string cmd = "rosrun simple_navigation_goals simple_navigation_goals " + arg1.str() 
                     + " " + arg2.str(); 
  system(cmd.c_str());
}

//MAIN
int main(int argc, char** argv){
  ros::init(argc, argv, "scheduler");
  ros::NodeHandle n;

  scheduler scheduler;
  //Aruco publishers and subscriptors
  ros::Subscriber aruco_id_sub = n.subscribe("aruco_simple/arucoID", 1, &scheduler::aruco_id_Callback, &scheduler);
  ros::Subscriber aruco_pose_sub = n.subscribe("aruco_simple/pose", 1, &scheduler::aruco_pose_Callback, &scheduler);
  ros::Subscriber yolo_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &scheduler::yolo_obj_found_Callback, &scheduler);
  scheduler.cancel_nav_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  scheduler.cancel_explore_pub = n.advertise<actionlib_msgs::GoalID>("explore_server/cancel",1);
  scheduler.tf_map_cam_client = n.serviceClient<simple_navigation_goals::getWorldPose>("get_world_pose");

  ros::Subscriber coin_bumper = n.subscribe("coin_bumper", 1000, &scheduler::coin_Callback, &scheduler);
  ros::Subscriber nav_status = n.subscribe("move_base/status", 1, &scheduler::navStatus_Callback, &scheduler);
  ros::Subscriber explore_status = n.subscribe("explore_server/status", 1, &scheduler::exploreStatus_Callback, &scheduler);

  //actionlib_msgs::GoalID stop_nav;

  
  ROS_INFO("Stage 1: Searching for the Aruco Marker...");
  //start servers and tf_echo (better in launch file)
  //system("rosrun simple_navigation_goals tf_echo_map_camera");
  //system("rosrun simple_navigation_goals object_mapping_server");
  //start navigation 
  //system("rosrun simple_navigation_goals simple_navigation_goals -17 17"); 

  scheduler.send_nav_goal(scheduler.aruco_pose[0], scheduler.aruco_pose[1]);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //Problem with coin
    scheduler.check_coin_issue();

    //STAGE 1
    if (scheduler.stage == 0)
    {
      if (scheduler.aruco_found == true) 
      {
        scheduler.robot_aruco_close();
        if (scheduler.aruco_close == true)
        {
          scheduler.cancel_nav_pub.publish(scheduler.stop_nav);
          ROS_INFO("Stage 2: Exploring the area to search for desired object...");
          system("rosrun frontier_exploration unbound_exploration");
          scheduler.stage = 1;
          n.setParam("/Object_ID", scheduler.desired_object);

          //Check if we have already detected the object previous to explore
          ROS_INFO("The desired object is the number %i", scheduler.desired_object);
          if (scheduler.obj_pos[scheduler.desired_object-1][0] == 1)
          {
            ROS_INFO("Object Already detected at [%f, %f]",
                     scheduler.obj_pos[scheduler.desired_object-1][3],
                     scheduler.obj_pos[scheduler.desired_object-1][4]);
          }
        }
      }
    }
  
    //STAGE 2 
    else if (scheduler.stage == 1)
    {
      //
    }

    
  
    //ROS_INFO("Canceling Aruco Node and Navigation...");
    //system("rosnode kill aruco_simple");
    //scheduler.cancel_nav_pub.publish(scheduler.stop_nav);
    //aruco_id_sub.shutdown();
    //aruco_pose_sub.shutdown();  

    //n.setParam("/Object_ID", scheduler.desired_object);


    //ROS_INFO("Stage 2: Exploring the area to search for desired object...");
    //system("rosrun frontier_exploration unbound_exploration");
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

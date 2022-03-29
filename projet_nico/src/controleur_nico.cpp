// COMMENTAIRE en 2spi
// la simulation sous gazebo marche pas
// je simule donc avec ritz avec la commande plus bas
// jai creer un service pour bouger les joints
//
// FONCTIONNEMENT
//
// 0. roslaunch niryo_one_gazebo gazebo_niryo_one.launch
//
// 1. lancer ritz dans un premier terminal :
// roslaunch niryo_one_bringup desktop_rviz_simulation.launch
//
// 2. lancer le run.launch dans un deuxieme terminal
// roslaunch projet_niryo_clement run.launch
//
// 3. lancer mon_service afin de bouger les joints du robot
// rosservice call /mon_service "{joint1: 3.0, joint2: 0.0, joint3: 0.0,
// joint4: 2.0, joint5: 5.0, joint6: 1.0}"

//#include "projet_niryo_clement/TrajectoireForwardKinematic.h"

// https://github.com/NiryoRobotics/niryo_one_ros/blob/master/niryo_one_python_api/src/niryo_one_python_api/niryo_one_api.py

#include "projet_niryo_clement/TrajectoireForwardKinematic.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>

#include "ros/duration.h"
#include "ros/subscriber.h"
#include <actionlib_msgs/GoalStatus.h>
#include <csignal>
#include <cstdio>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <geometry_msgs/PoseStamped.h>
#include <projet_nico/Position.h>
//#include <niryo_one_commander/command_type.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/RobotMoveGoal.h>

// photo
#include <niryo_one_msgs/TakePicture.h>
#include <niryo_one_msgs/TakePictureResponse.h>
// commpresse image
#include <sensor_msgs/CompressedImage.h>
// from niryo_one_commander.command_type import CommandType as MoveCommandType
//#include <opencv-3.3.1-dev/opencv2/opencv.hpp> //l'import marche

//#include <cv_bridge/cv_bridge.h>
//#include <opencv4/opencv2/opencv.hpp>
//#include <opencv4/opencv2/core.hpp>
//#include <opencv_apps/>
//#include <opencv2/videoio.hpp

//#include <opencv-3.3.1-dev/opencv2/core.hpp>
//#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
//#include <opencv-3.3.1-dev/opencv2/videoio.hpp>
//#include <opencv-3.3.1-dev/opencv2/highgui.hpp>
//#include <opencv-3.3.1-dev/opencv2/imgproc.hpp>

#define FREQUENCY 1

using namespace ros;
using namespace std;
using namespace std_msgs;
using namespace actionlib_msgs;
using namespace trajectory_msgs;
using namespace niryo_one_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
// using namespace cv;

typedef actionlib::SimpleActionClient<RobotMoveAction> Client;
typedef actionlib::SimpleActionServer<RobotMoveAction> Server;

// Prototypes des fonctions
void sigintHandler(int sig);

// Variables globales
Publisher log_pub;
Subscriber trajectoire_sub;
Subscriber compressed_image_sub;
Subscriber position_sub;

JointTrajectory join_trajectory;
JointTrajectoryPoint joint_trajectory_point;

CompressedImage ImageCamera;
int x;
int y;
int couleur;

double frequence = FREQUENCY;

// Fonctions
void move_pose(float x, float y, float z, float roll, float pitch, float yaw) {
  RobotMoveActionGoal goal;

  goal.goal.cmd.position.x = x;
  goal.goal.cmd.position.y = y;
  goal.goal.cmd.position.z = z;

  goal.goal.cmd.rpy.roll = roll;
  goal.goal.cmd.rpy.pitch = pitch;
  goal.goal.cmd.rpy.yaw = yaw;

  goal.goal.cmd.cmd_type = 2;

  Client client("/niryo_one/commander/robot_action/", true);
  ROS_INFO("Action server started, sending goal.");
  client.sendGoal(goal.goal);

  client.waitForResult(ros::Duration(2.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot is moving");
  }
  // ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
}

// Callback
void trajectoireCallback(JointTrajectory join_trajectory) {
  for (int i = 0; i < 6; i++) {
    ROS_INFO("Joint %d , position : %f", i + 1,
             joint_trajectory_point.positions[i]);
  }
}
void compressed_image_Callback(CompressedImage compresse_image) {
  ImageCamera = compresse_image;
  ROS_INFO("NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNUUUUUUULLL");
  // ROS_INFO("image recu : %s", ImageCamera.format.c_str());
}
void positionCallback(projet_nico::Position data) {
  // ROS_INFO("aaaaaaaaaa");
  x = data.x;
  y = data.y;
  couleur = data.color;
  ROS_INFO("X: %d Y:%d couleur :%d  ", data.x, data.y, data.color);
}

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;
  std::string espace;

  // publishers
  trajectoire_sub = nh.subscribe("/niryo_one_follow_joint_trajectory_\
controller/command",
                                 1000, trajectoireCallback);

  compressed_image_sub =
      nh.subscribe("/niryo_one_vision/compressed_video_stream/", 1000,
                   compressed_image_Callback);
  position_sub = nh.subscribe("/position_topic", 1000, positionCallback);
  Rate loop_rate(frequence);

  // ------------- CONNECTION AU ROBOT --------------------

  ROS_INFO("---------- Connecting to robot -----------");
  Client client("/niryo_one/commander/robot_action/", true);

  // wait for the action server to start
  bool connection_success = false;
  while (!connection_success) {
    connection_success = client.waitForServer(ros::Duration(3.0));
    if (connection_success) {
      ROS_INFO("  Robot Connection established");
    } else {
      ROS_WARN("  Error connecting to Robot. Trying again");
    }
  }
  move_pose(0.224, 0.015, 0.147, 0.086, 1.123, 0.054);
  /*
  VideoCapture cap(0); // open the default camera

  if (!cap.isOpened())  // check if we succeeded
      return -1;

  Mat frame;
  */

  // ----------------- BOUCLE INFINIE ------------------
  while (ok()) {

    // ROS_INFO("  Press enter to send ... pose 1");
    //       ROS_INFO_STREAM_ONCE()
    // getline(std::cin, espace);

    switch (couleur) {
    case 1:

    case 3:
      move_pose(0, 0.256, 0.129, 0.081, 1.116, 1.566); // bleu
      ros::Duration a(1);
      move_pose(0.224, 0.015, 0.147, 0.086, 1.123, 0.054);
    }
    couleur = 0;

    /*move_pose(0, 0.256, 0.129, 0.081, 1.116, 1.566);

    ROS_INFO("  Press enter to send ...pose 2");
    getline(std::cin, espace);
    move_pose(0.224, 0.015, 0.147, 0.086, 1.123, 0.054);*/

    spinOnce();
    loop_rate.sleep();
    /*
    cap >> frame;
    imshow("frame", frame);


    */
  }
  return 0;
}

// FIN DU PROGRAMME  FIN DU PROGRAMME  FIN DU PROGRAMME  FIN DU PROGRAMME

/**
 *
 * SIGINT handler.
 * Exit code gracefully.
 *
 * @param sig - catched signal.
 *
 */

void sigintHandler(int sig) {
  // Log quit
  ROS_INFO("Exiting program gracefully ...");

  // MESSAGE A PUBLIER le MESSAGE

  // Kill all open subscriptions, publications, service calls, and service
  // servers
  shutdown();
}
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tue_manipulation_msgs/GripperCommandAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_client");

  if (argc != 3) {
	  ROS_ERROR("Arguments: SIDE (\"left\" or \"right\") DIRECTION (\"open\" or \"close\")");
	  return 1;
  }

  std::string side = argv[1];
  std::string cmd = argv[2];

  // create the action client
  std::stringstream server_name;
  server_name << "/gripper_server_" << side;
  actionlib::SimpleActionClient<tue_manipulation_msgs::GripperCommandAction> ac(server_name.str());
  ac.waitForServer();

  // send a goal to the action
  tue_manipulation_msgs::GripperCommandGoal goal;

  if (cmd == "open") {
      goal.command.direction = tue_msgs::GripperCommand::OPEN;
  } else if (cmd == "close") {
      goal.command.direction = tue_msgs::GripperCommand::CLOSE;
  } else {
	  ROS_ERROR("Goal should be either \"open\" or \"close\".");
	  return 1;
  }

  goal.command.max_force = 10;

  while(ros::ok()) {

  	  actionlib::SimpleClientGoalState state = ac.sendGoalAndWait(goal, ros::Duration(5));
  	  ROS_WARN("Current state: %s", state.toString().c_str());

  	goal.command.direction = -goal.command.direction;
  }

  /*
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)   {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
	actionlib::SimpleClientGoalState state = ac.getState();
    ROS_WARN("Action did not finish before the time out: %s", state.toString().c_str());
  }
  */

  ros::shutdown();

  return 0;
}

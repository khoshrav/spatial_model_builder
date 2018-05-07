#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <umass_control_msgs/ControlAction.h>
#include <umass_control_msgs/ControlResult.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gaze_control_client");

  ros::NodeHandle n_;

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<umass_control_msgs::ControlAction> client("GC", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer(); //will wait for infinite time
  umass_control_msgs::ControlGoal goal;
  goal.head_track_mode = 2;

  if (argc > 1)
  {

    goal.center_goal.position.x = atof(argv[1]);
    goal.center_goal.position.y = atof(argv[2]);
    goal.center_goal.position.z = atof(argv[3]);
  }
  else
  {
    goal.center_goal.position.x = 0.500;
    goal.center_goal.position.y = 0.00;
    goal.center_goal.position.z = 0.00;
  }



  goal.reference ="base_footprint";
  client.sendGoal(goal);
  ROS_INFO("Sent goal, waiting for controller...");

  if (client.waitForResult(ros::Duration(30.0)))
  {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Controller finished: %s", state.toString().c_str());

  }

  client.cancelAllGoals();

  //exit
  return 0;
}

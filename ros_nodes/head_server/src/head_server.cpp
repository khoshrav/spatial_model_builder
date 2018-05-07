#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "head_server/headmanager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_server");
  ros::NodeHandle n("~");

  HeadManager_ptr hm_ptr(new HeadManager(n));
  ros::Rate rate(200.0);

  while(ros::ok()) {
    ros::spinOnce();
    hm_ptr->updateDT();
    hm_ptr->readDynamixelState();
    hm_ptr->updateDynamixelState();
    hm_ptr->publishStates();
    hm_ptr->publishDesiredStates();
    rate.sleep();
  }
  return 0;

}

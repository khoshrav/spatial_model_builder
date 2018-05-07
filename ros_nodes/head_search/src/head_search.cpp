#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "head_search/searchmanager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_search");
  ros::NodeHandle n("~");

  Search sh(n);
  ros::Rate rate(0.1);
  while(ros::ok()) {
    ros::spinOnce();
    sh.publishState();
    sh.gazeRandomSample();
    sh.saveAllModels();
    rate.sleep();
  }
  return 0;

}

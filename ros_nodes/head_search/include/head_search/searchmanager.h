#ifndef SEARCHMANAGER_H_
#define SEARCHMANAGER_H_
#include <map>
#include <ros/ros.h>
#include <darknet_ros/bbox_array.h>
#include <math.h>
#include <set>
#include <tuple>
#include <ctime>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <umass_control_msgs/ControlAction.h>
#include <umass_control_msgs/ControlResult.h>
#include <boost/range/iterator_range.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "head_search/entity.h"
#include "head_search/FindEntity.h"

class Search
{
public:
  Search(ros::NodeHandle node_handle);
  ~Search();
  void publishState();
  void gazeRandomSample();
  void saveAllModels();
private:
  ros::NodeHandle n_;
  ros::Subscriber found_object_sub;
  ros::Subscriber change_state_sub;
  ros::ServiceServer find_service;
  ros::Publisher search_state_pub;
  void changeStateCallback(const std_msgs::Int32ConstPtr& msg);
  void yoloCallback(const darknet_ros::bbox_arrayConstPtr& msg);
  bool FindService(head_search::FindEntity::Request& req, head_search::FindEntity::Response& res);
  int RandomSearch(std::string class_name);
  std::map<std::string,Entity> objectDistributions;
  // std::map<const char*,Entity, StrCompare> objectDistributions;
  enum possible_states
  {
    build_model =0,
    query_model,
    searching,
    found,
    not_found
  };
  enum possible_states search_state;
  std::string  search_target;
  void gazeAtPoint(double x, double y, double z);
  double found_x;
  double found_y;
  double found_z;
  double found_likelihood;
  std::vector<std::string> search_state_verbose;
  std::string log_path;
  bool loadModels;
};

#endif

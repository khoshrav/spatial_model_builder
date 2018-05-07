#include "head_search/searchmanager.h"
#include <iterator>

Search::Search(ros::NodeHandle node_handle): n_(node_handle)
{
    srand(time(NULL));
    found_object_sub = n_.subscribe<darknet_ros::bbox_array>("/yolo_ros/YOLO_bboxes", 10,
                        &Search::yoloCallback, this);
    find_service = n_.advertiseService("find_entity", &Search::FindService, this);
    search_state_verbose.push_back("Build Model");
    search_state_verbose.push_back("Query Model");
    search_state_verbose.push_back("Searching");
    search_state_verbose.push_back("Found");
    search_state_verbose.push_back("Not Found");

    search_state_pub = n_.advertise<std_msgs::String>("/head_search/state", 1);
    change_state_sub = n_.subscribe<std_msgs::Int32>("/head_search/change_state", 1,
                        &Search::changeStateCallback, this);
    search_state = build_model;
    if (!n_.getParam("/head_search/log_path", log_path))
    {
      log_path = "/home/khoshrav/data/head_search/";
      ROS_WARN("Could not retrieve 'log_path' parameter, setting to default value: '%s' \n",
               log_path.c_str());
    }
    if (!n_.getParam("/head_search/loadModels", loadModels))
    {
      loadModels = false;
      ROS_WARN("Could not retrieve 'loadModels' parameter, setting to default value: '%s' \n",
               (loadModels ? "true" : "false"));
    }

    if(loadModels)
    {
      boost::filesystem::path model_dir(log_path);
      if(boost::filesystem::is_directory(model_dir))
      {
        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(model_dir), {}))
        {
          printf("Loading %s", entry.path().filename().string().c_str());
          Entity obj(entry.path().filename().string(), log_path);
          obj.loadModel();
          objectDistributions.insert(std::pair<std::string, Entity>(entry.path().filename().string(), obj));
        }
      }
    }
    search_state = query_model;
}
Search::~Search()
{
  saveAllModels();
}

void Search::saveAllModels()
{
  if(search_state == build_model)
  {
    for (std::map<std::string,Entity>::iterator it=objectDistributions.begin();
      it!=objectDistributions.end();
      ++it)
      {
        ROS_INFO("Saving model for %s", it->first.c_str());
        it->second.saveModel();
      }
  }
}

void Search::publishState()
{
  std_msgs::String msg;
  msg.data = search_state_verbose.at(search_state);
  search_state_pub.publish(msg);
}

void Search::changeStateCallback(const std_msgs::Int32ConstPtr& msg)
{
  search_state = static_cast<possible_states>(msg->data);
}

// Receive a message from darknet_yolo
void Search::yoloCallback(const darknet_ros::bbox_arrayConstPtr& msg)
{
  // For each object in the message
  for(size_t i = 0; i < msg->bboxes.size(); i++)
  {
      const char* temp_name = msg->bboxes[i].Class.c_str();
      std::string class_name(temp_name);
      double prop =  msg->bboxes[i].prob;
      double x, y, z;
      // Discretize into 0.1m bins
      x = roundf(msg->bboxes[i].x * 10) / 10;
      y = roundf(msg->bboxes[i].y * 10) / 10;
      z = roundf(msg->bboxes[i].z * 10) / 10;
      if (isnan(x) || isnan(y) || isnan(z))
      {
        continue;
      }
      if (search_state == build_model)
      {
        std::map<std::string, Entity>::iterator it = objectDistributions.find(class_name);

        if(it != objectDistributions.end())
        {
          // We have seen the entity before
          it->second.observedAt(prop, x, y, z);
          ROS_INFO("Found %s (%lf) at (%lf, %lf, %lf)", class_name.c_str(), prop, x, y, z);
        }
        else
        {
          // Create new entity
          Entity obj(class_name, log_path);
          obj.observedAt(prop, x, y, z);
          std::pair<std::map<std::string, Entity>::iterator,bool> ret;
          ret = objectDistributions.insert(std::pair<std::string, Entity>(class_name, obj));
          ROS_INFO("ADDING NEW %s (%lf) at (%lf, %lf, %lf)\n",
           class_name.c_str(), prop, x, y, z);
        }

      }
      else if(search_state == searching)
      {
        // ROS_INFO("Searching for %s, got %s", search_target.c_str(), class_name.c_str());
        if(search_target == class_name)
        {
          search_state = found;
          found_x =x;
          found_y =y;
          found_z =z;
          found_likelihood = prop;
          ROS_ERROR("FOUND %s at %lf %lf %lf", search_target.c_str(), found_x, found_y, found_z);
          gazeAtPoint(found_x, found_y, found_z);
        }
      }

      fflush(stdout);
  }
}


bool Search::FindService(head_search::FindEntity::Request& req, head_search::FindEntity::Response& res)
{
  ROS_INFO("Seaching for %s (mode = %d)", req.EntityClass.c_str(), req.useProbability);
  search_state = query_model;
  std::string class_name = req.EntityClass;

  /*
  // Uncomment for uniform prior search
  int counter  = RandomSearch(class_name);
  if(search_state == found)
  {
    ROS_INFO("Found entity %s at (%lf, %lf, %lf) in %d actions",
      class_name.c_str(), found_x, found_y, found_z, counter);
    res.x = found_x;
    res.y = found_y;
    res.z = found_z;
    res.likelihood = found_likelihood;
    res.actionsTaken = counter;
    return true;
  }*/


  // Comment from here for uniform prior search
  std::multimap<long double, std::tuple<double, double, double>> s;
  std::map<std::string, Entity>::iterator it = objectDistributions.find(class_name);
  if(it == objectDistributions.end())
  {
    // Entity has not been seen
    // TODO: Perform random search
    ROS_INFO("%s not present in seen objects", class_name.c_str());
    // printf("NOT FOUND BVALALALAL\n");
    return false;
  }
  if (static_cast<bool>(req.useProbability) == false)
  {
    // Use count based voxel
    s = objectDistributions[class_name].find();
  }
  else
  {
    // Use probability based voxel
    s = objectDistributions[class_name].findProb();
  }

  int counter =1;
  search_target = class_name;
  search_state = searching;
  for(auto it : s)
      {
        if(search_state == searching)
        {
          break;
        }
        // Get position model wants us to look at
        std::tuple<double, double, double> tu = (it).second;
        gazeAtPoint(roundf(std::get<0>(tu) * 10) / 10,
          roundf(std::get<1>(tu) * 10) / 10,
          roundf(std::get<2>(tu) * 10) / 10);
        printf("Prob %d (%lf, %lf, %lf) = %lf\n", counter, std::get<0>(tu), std::get<1>(tu), std::get<2>(tu), (it).first);
        // Receive messages from darknet_ros
        ros::spinOnce();
        publishState();
        if(search_state == found)
        {
          ROS_INFO("Found entity %s at (%lf, %lf, %lf) in %d actions. Was looking for (%lf, %lf, %lf)",
            class_name.c_str(), found_x, found_y, found_z, counter,
            roundf(std::get<0>(tu) * 10) / 10,
            roundf(std::get<1>(tu) * 10) / 10,
            roundf(std::get<2>(tu) * 10) / 10);
          res.x = found_x;
          res.y = found_y;
          res.z = found_z;
          res.likelihood = found_likelihood;
          res.actionsTaken = counter;
          return true;
        }
        counter++;
      }
    search_state = not_found;
    // Comment till here for uniform prior search

    ROS_INFO("NOT found %s in %d actions", class_name.c_str(), counter);
    return false;
}

// Sample random x y z and send gaze command
void Search::gazeRandomSample()
{
  if(search_state == build_model)
  {
    double x_goal, y_goal, z_goal;
    x_goal = static_cast<double>(rand())/RAND_MAX;
    y_goal = static_cast<double>(2*rand())/RAND_MAX;
    z_goal = static_cast<double>(rand())/RAND_MAX;
    gazeAtPoint(x_goal, y_goal, z_goal);
  }
}

// Interface with gaze_controller
void Search::gazeAtPoint(double x_goal, double y_goal, double z_goal)
{
  umass_control_msgs::ControlGoal goal;
  goal.head_track_mode = 2;

  goal.center_goal.position.x = x_goal;
  goal.center_goal.position.y = y_goal;
  goal.center_goal.position.z = z_goal;

  goal.reference ="base_footprint";

  actionlib::SimpleActionClient<umass_control_msgs::ControlAction> gaze_client("GC", true);
  gaze_client.waitForServer(); //will wait for infinite time
  ROS_INFO("Sending to: %lf \t %lf \t %lf", x_goal, y_goal, z_goal);
  gaze_client.sendGoalAndWait(goal);
}

// Uniform prior search
int Search::RandomSearch(std::string class_name)
{
  search_target = class_name;
  search_state = searching;
  int counter = 1;
  while(search_state == searching)
  {
    double x_goal, y_goal, z_goal;
    x_goal = static_cast<double>(rand())/RAND_MAX;
    y_goal = static_cast<double>(2*rand())/RAND_MAX;
    z_goal = static_cast<double>(rand())/RAND_MAX;
    gazeAtPoint(x_goal, y_goal, z_goal);
    ros::spinOnce();
    publishState();
    ROS_WARN("woke up!");
    if(search_state == found)
    {
      break;
    }
    counter++;
    if(counter >=25)
    {
      search_state = not_found;
      break;
    }
  }
  return counter;
}

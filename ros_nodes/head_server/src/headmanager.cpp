#include <head_server/headmanager.h>

HeadManager::HeadManager(ros::NodeHandle node_handle) :
    n_(node_handle)
{
  ROS_INFO("HeadManager Constructor");

  loopback_mode_ = false;
  if (!n_.getParam("/head_server/loopback", loopback_mode_))
  {
    ROS_WARN("Could not retrieve 'loopback' parameter, setting to default value: '%s' \n",
             (loopback_mode_ ? "true" : "false"));
  }
  if (loopback_mode_)
  {
    ROS_WARN("Running in loopback mode! Position commands will not be sent to robot but reflected directly.");
  }

  gazebo_sim_mode_ = false;
  if (!n_.getParam("/head_server/gazebo_simulation", gazebo_sim_mode_))
  {
    ROS_WARN("Could not retrieve 'gazebo_simulation' parameter, setting to default value: '%s' \n",
             (gazebo_sim_mode_ ? "true" : "false"));
  }
  if (gazebo_sim_mode_)
  {
    loopback_mode_ = false;
    ROS_WARN("Running in simulation mode with gazebo!");
  }

  ROS_INFO("Loopback_mode: %s", (loopback_mode_ ? "true" : "false"));
  ROS_INFO("Gazebo_sim_mode: %s", (gazebo_sim_mode_ ? "true" : "false"));

  dt_ = 0.0;
  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();

  joint_names_.push_back("xtion_head_upper_tilt_joint");
  joint_names_.push_back("xtion_head_pan_joint");
  joint_names_.push_back("xtion_head_lower_tilt_joint");

  all_motor_limits.resize(NUMBER_OF_MOTORS);

  urdf::Model robot_description;
  if (!robot_description.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
  }
  ROS_INFO("Successfully parsed urdf file");

  // all_motor_limits[LOWER_HEAD_TILT].min_radian = -1.57;
  // all_motor_limits[LOWER_HEAD_TILT].max_radian = 1.57;
  //
  // all_motor_limits[HEAD_PAN].min_radian = -2.9;
  // all_motor_limits[HEAD_PAN].max_radian = 2.9;
  //
  // all_motor_limits[UPPER_HEAD_TILT].min_radian = -1.3;
  // all_motor_limits[UPPER_HEAD_TILT].max_radian = 1.3;

  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    urdf::JointConstSharedPtr joint = robot_description.getJoint(joint_names_[MOTOR_INDEX]);
    all_motor_limits[MOTOR_INDEX].min_radian = joint->limits->lower;
    all_motor_limits[MOTOR_INDEX].max_radian = joint->limits->upper;

    std::string motorString = "/" + joint_names_.at(MOTOR_INDEX) + "_controller/motor/zero_tick";
    all_motor_limits[MOTOR_INDEX].zero_value = n_.param<int>(motorString, 2048);
    double radian_range = all_motor_limits[MOTOR_INDEX].max_radian - all_motor_limits[MOTOR_INDEX].min_radian;
    int tick_range = VALUES_PER_RADIAN * radian_range;
    all_motor_limits[MOTOR_INDEX].min_value = std::max(all_motor_limits[MOTOR_INDEX].zero_value - (tick_range/2),
                                                      static_cast<unsigned int>(MIN_VALUE));
    all_motor_limits[MOTOR_INDEX].max_value = std::min(all_motor_limits[MOTOR_INDEX].zero_value + (tick_range/2),
                                                      static_cast<unsigned int>(MAX_VALUE));
    ROS_INFO("-------------------------");
    ROS_INFO("MOTOR_INDEX = %d", static_cast<int>(MOTOR_INDEX));
    ROS_INFO("jointname = %s", joint_names_[MOTOR_INDEX].c_str());
    ROS_INFO("MotorStr = %s", motorString.c_str());
    ROS_INFO("min rad = %f", all_motor_limits[MOTOR_INDEX].min_radian);
    ROS_INFO("max rad = %f", all_motor_limits[MOTOR_INDEX].max_radian);
    ROS_INFO("zero tick = %d", all_motor_limits[MOTOR_INDEX].zero_value);
    ROS_INFO("min tick = %d", all_motor_limits[MOTOR_INDEX].min_value);
    ROS_INFO("max tick = %d", all_motor_limits[MOTOR_INDEX].max_value);

    ROS_INFO("-------------------------");
  }


  // TODO: Load default values from yaml
  positions_.resize(NUMBER_OF_MOTORS, 0.0);
  desired_positions_.resize(NUMBER_OF_MOTORS, 0.0);

  joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("/uBot_head/joint_states", 1);
  desired_states_pub_ = n_.advertise<sensor_msgs::JointState>("/uBot_head/joint_desired_states", 1);
  dt_pub_ = n_.advertise<std_msgs::Float32>("/uBot_head/loop_dt", 1);

  head_command_sub_ = n_.subscribe<sensor_msgs::JointState>("/uBot_head/command", 1,
                                                          &HeadManager::commandCallback, this);

  if (!loopback_mode_ && !gazebo_sim_mode_)
  {
    if(loadDynamixel())
    {
      ROS_INFO("Connected to All dynamixel motors");
    }
    else
    {
      ROS_ERROR("Could not connect to Dynamixel Motors");
      checkLoadDynamixel();
      // exit(10);
    }
    if (!multi_driver_->initSyncWrite())
    {
      ROS_ERROR("Init SyncWrite Failed!");
    }

    writeValue_ = new WriteValue;
    setDesiredToCurrent();
    // setTorque(true);

    // if (multi_driver_->getProtocolVersion() == 2.0 &&
    //     !(multi_driver_->multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
    // {
    //   setProfileValue(profile_velocity_, profile_acceleration_);
    // }
  }
}

HeadManager::~HeadManager() {}

void HeadManager::setDesiredToCurrent()
{
  if (!loopback_mode_ && !gazebo_sim_mode_)
  {
    readDynamixelState();
    updateDynamixelState();
  }
  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    desired_positions_[MOTOR_INDEX] = positions_[MOTOR_INDEX];
  }
}


bool HeadManager::loadDynamixel()
{
  bool ret = false;
  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    dynamixel_driver::DynamixelInfo *motorInfo = new dynamixel_driver::DynamixelInfo;
    motorInfo->lode_info.device_name = n_.param<std::string>("/device_name", "/dev/ttyUSB0");
    motorInfo->lode_info.baud_rate        = n_.param<int>("/baud_rate", 1000000);
    motorInfo->lode_info.protocol_version = n_.param<float>("/protocol_version", 2.0);
    std::string idString = "/" + joint_names_.at(MOTOR_INDEX) + "_controller/motor/id";
    motorInfo->model_id                   = n_.param<int>(idString, static_cast<int>(MOTOR_INDEX));

    ROS_INFO("___________________________");
    ROS_INFO("MOTOR_INDEX = %d",static_cast<int>(MOTOR_INDEX));
    ROS_INFO("Device Name = %s",motorInfo->lode_info.device_name.c_str());
    ROS_INFO("Baud Rate = %d",motorInfo->lode_info.baud_rate);
    ROS_INFO("Protocol Version = %f",motorInfo->lode_info.protocol_version);
    ROS_INFO("ID  = %d",motorInfo->model_id);
    ROS_INFO("___________________________");
    dynamixel_info_.push_back(motorInfo);
  }

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[DYNAMIXEL_ZERO]->lode_info.device_name,
                                                                   dynamixel_info_[DYNAMIXEL_ZERO]->lode_info.baud_rate,
                                                                   dynamixel_info_[DYNAMIXEL_ZERO]->lode_info.protocol_version);


 ret = multi_driver_->loadDynamixel(dynamixel_info_);
 return ret;
}

void HeadManager::updateDT()
{
  //calculate dt
  current_time_ = ros::Time::now();
  dt_ = (current_time_ - last_time_).toSec();

  std_msgs::Float32 msg;
  msg.data = dt_;

  //TODO: remove this or make thread safe...
  // msg.data = 1.0 / com_loop_dt_;
  dt_pub_.publish(msg);
}

void HeadManager::publishStates()
{
  if(joint_states_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.resize(NUMBER_OF_MOTORS, "");
  msg.position.resize(NUMBER_OF_MOTORS, 0.0);
  // msg.velocity.resize(NUMBER_OF_MOTORS, 0.0);
  // msg.effort.resize(NUMBER_OF_MOTORS, 0.0);
  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    msg.name[MOTOR_INDEX] = joint_names_[MOTOR_INDEX];
    msg.position[MOTOR_INDEX] = positions_[MOTOR_INDEX];
  }
  joint_states_pub_.publish(msg);
}

void HeadManager::publishDesiredStates()
{
  if(desired_states_pub_.getNumSubscribers() == 0)
  {
    return;
  }
  // if (!loopback_mode_ && !gazebo_sim_mode_)
  // {
  //   ROS_INFO("In publish desired states and read&update");
  //   readDynamixelState();
  //   updateDynamixelState();
  // }
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.resize(NUMBER_OF_MOTORS, "");
  msg.position.resize(NUMBER_OF_MOTORS, 0.0);
  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    msg.name[MOTOR_INDEX] = joint_names_[MOTOR_INDEX];
    msg.position[MOTOR_INDEX] = desired_positions_[MOTOR_INDEX];
  }
  desired_states_pub_.publish(msg);
}

void HeadManager::readDynamixelState()
{
  if (loopback_mode_ || gazebo_sim_mode_) {
    return;
  }

  multi_driver_->readMultiRegister("present_position");

  // multi_driver_->readMultiRegister("torque_enable");
  // multi_driver_->readMultiRegister("goal_position");


  // multi_driver_->readMultiRegister("moving");
  //
  // if (multi_driver_->getProtocolVersion() == 2.0)
  // {
  //   if (multi_driver_->multi_dynamixel_[DYNAMIXEL_ZERO]->model_name_.find("XM") != std::string::npos)
  //   {
  //     multi_driver_->readMultiRegister("goal_current");
  //
  //     multi_driver_->readMultiRegister("present_current");
  //   }
  //   multi_driver_->readMultiRegister("goal_velocity");
  //   multi_driver_->readMultiRegister("present_velocity");
  // }
  // else
  // {
  //   multi_driver_->readMultiRegister("moving_speed");
  //   multi_driver_->readMultiRegister("present_speed");
  // }
}

void HeadManager::updateDynamixelState()
{
  if (loopback_mode_ || gazebo_sim_mode_) {
    return;
  }
  if (multi_driver_->multi_dynamixel_.size() != NUMBER_OF_MOTORS)
  {
    ROS_ERROR("Cannot update state of all motors");
    return;
  }
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type MOTOR_INDEX = 0; MOTOR_INDEX < multi_driver_->multi_dynamixel_.size(); ++MOTOR_INDEX)
  {
    positions_[MOTOR_INDEX] = convertValue2Radian(multi_driver_->read_value_["present_position"]->at(MOTOR_INDEX), MOTOR_INDEX);
    // if (MOTOR_INDEX == HEAD_PAN)
    // {
    //   positions_[MOTOR_INDEX] = convertValue2Radian(multi_driver_->read_value_["present_position"]->at(MOTOR_INDEX) * HEAD_PAN_REDUCTION ,
    //                               MOTOR_INDEX);
    // }
    // ROS_INFO("current raw[%d] = %d", static_cast<int>(MOTOR_INDEX), multi_driver_->read_value_["present_position"]->at(MOTOR_INDEX));
    // ROS_INFO("current radian[%d] = %f", static_cast<int>(MOTOR_INDEX), positions_[MOTOR_INDEX]);
  }
  positions_[HEAD_PAN] = positions_[HEAD_PAN] * HEAD_PAN_REDUCTION;
  // ROS_INFO("current raw[%d] = %d", static_cast<int>(HEAD_PAN), multi_driver_->read_value_["present_position"]->at(HEAD_PAN));
  // ROS_INFO("current radian[%d] = %f", static_cast<int>(HEAD_PAN), positions_[HEAD_PAN]);
}

void HeadManager::commandCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (size_t MSG_INDEX = 0; MSG_INDEX < msg->name.size(); MSG_INDEX++)
    {
      size_t MOTOR_INDEX = UPPER_HEAD_TILT;
      for (; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++) {
        if(joint_names_[MOTOR_INDEX] == msg->name[MSG_INDEX])
        {
          break;
        }
      }
      if (MOTOR_INDEX == NUMBER_OF_MOTORS) {
        continue;
      }
      // ROS_INFO("msg[%d] = %s && motor[%d] = %s && pos = %f",
      //         static_cast<int>(MSG_INDEX), msg->name[MSG_INDEX].c_str(),
      //         static_cast<int>(MOTOR_INDEX), joint_names_[MOTOR_INDEX].c_str(),
      //       msg->position[MSG_INDEX]);
      //obey limits
      double pos = msg->position[MSG_INDEX];

      pos = std::min(static_cast<double>(all_motor_limits[MOTOR_INDEX].max_radian), pos);
      pos = std::max(static_cast<double>(all_motor_limits[MOTOR_INDEX].min_radian), pos);
      desired_positions_[MOTOR_INDEX] = pos;
      if (loopback_mode_ || gazebo_sim_mode_)
      {
        positions_[MOTOR_INDEX] = pos;
      }
    }
    if (!loopback_mode_ && !gazebo_sim_mode_)
    {
      writePositions();
    }
}

void HeadManager::writePositions()
{
  if (loopback_mode_ || gazebo_sim_mode_)
  {
    return;
  }
  writeValue_->pos.clear();
  for (size_t MOTOR_INDEX = UPPER_HEAD_TILT; MOTOR_INDEX < NUMBER_OF_MOTORS; MOTOR_INDEX++)
  {
    if (MOTOR_INDEX == HEAD_PAN)
    {
      writeValue_->pos.push_back(convertRadian2Value((desired_positions_[MOTOR_INDEX] / HEAD_PAN_REDUCTION), MOTOR_INDEX));
      // desired_positions_[MOTOR_INDEX] = desired_positions_[MOTOR_INDEX] / HEAD_PAN_REDUCTION;
    }
    else
    {
      writeValue_->pos.push_back(convertRadian2Value(desired_positions_[MOTOR_INDEX], MOTOR_INDEX));
    }
  }
  multi_driver_->syncWritePosition(writeValue_->pos);
}

void HeadManager::checkLoadDynamixel()
{
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; position control example(Pan & Tilt) ");
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("LOWER_HEAD_TILT");
  ROS_INFO("ID    : %d", dynamixel_info_[LOWER_HEAD_TILT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[LOWER_HEAD_TILT]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("HEAD_PAN");
  ROS_INFO("ID    : %d", dynamixel_info_[HEAD_PAN]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[HEAD_PAN]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("UPPER_HEAD_TILT");
  ROS_INFO("ID    : %d", dynamixel_info_[UPPER_HEAD_TILT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[UPPER_HEAD_TILT]->model_name.c_str());
  // if (multi_driver_->getProtocolVersion() == 2.0 &&
  //     !(multi_driver_->multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
  // {
  //   ROS_INFO(" ");
  //   ROS_INFO("Profile Velocity     : %d", profile_velocity_);
  //   ROS_INFO("Profile Acceleration : %d", profile_acceleration_);
  // }
  ROS_INFO("-----------------------------------------------------------------------");
}

double HeadManager::convertValue2Radian(int32_t value, int MOTOR_INDEX)
{
  double radian = 0.0;
  if (value > all_motor_limits[MOTOR_INDEX].max_value)
  {
    return all_motor_limits[MOTOR_INDEX].max_radian;
  }
  else if (value < all_motor_limits[MOTOR_INDEX].min_value)
  {
    return all_motor_limits[MOTOR_INDEX].min_radian;
  }
  else
  {
    radian = ((value - all_motor_limits[MOTOR_INDEX].min_value) * RADIANS_PER_VALUE) + all_motor_limits[MOTOR_INDEX].min_radian;
    radian = std::max(all_motor_limits[MOTOR_INDEX].min_radian, radian);
    radian = std::min(all_motor_limits[MOTOR_INDEX].max_radian, radian);
  }
  return radian;
}

uint32_t HeadManager::convertRadian2Value(float radian, int MOTOR_INDEX)
{
  uint32_t value = 0;
  if (radian < all_motor_limits[MOTOR_INDEX].min_radian)
  {
    return all_motor_limits[MOTOR_INDEX].min_value;
  }
  if (radian > all_motor_limits[MOTOR_INDEX].max_radian)
  {
    return all_motor_limits[MOTOR_INDEX].max_value;
  }
  else
  {
    value = (radian * VALUES_PER_RADIAN) + all_motor_limits[MOTOR_INDEX].zero_value;
    value = std::max(all_motor_limits[MOTOR_INDEX].min_value, value);
    value = std::min(all_motor_limits[MOTOR_INDEX].max_value, value);
  }
}

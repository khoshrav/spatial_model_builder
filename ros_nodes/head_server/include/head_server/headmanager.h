#ifndef THREE_DOF_HEAD_INCLUDE_THREE_DOF_HEAD_HEADMANAGER_H_
#define THREE_DOF_HEAD_INCLUDE_THREE_DOF_HEAD_HEADMANAGER_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <vector>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

#include <urdf/model.h>

#define HEAD_PAN_REDUCTION (75.0 / 66.0)
#define DYNAMIXEL_ZERO 0
#define MIN_VALUE 0
#define MAX_VALUE 4096
#define MIN_RADIAN 0.0
#define MAX_RADIAN 6.28319
#define VALUES_PER_RADIAN ((MAX_VALUE - MIN_VALUE) / (MAX_RADIAN - MIN_RADIAN))
#define RADIANS_PER_VALUE ((MAX_RADIAN - MIN_RADIAN) / (MAX_VALUE - MIN_VALUE))
// #define NUMBER_OF_MOTORS 2
enum HeadJoints {
  UPPER_HEAD_TILT = 0,
  HEAD_PAN,
  LOWER_HEAD_TILT,
  NUMBER_OF_MOTORS
};

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
  std::vector<uint32_t> prof_vel;
  std::vector<uint32_t> prof_acc;
}WriteValue;

struct motor_limits_params
{
  double min_radian;
  double max_radian;
  uint32_t min_value;
  uint32_t max_value;
  uint32_t zero_value;
};

class HeadManager;
typedef boost::shared_ptr<HeadManager> HeadManager_ptr;

class HeadManager
{
public:
  HeadManager(ros::NodeHandle node_handle);
  virtual ~HeadManager();

  void updateDT();
  void publishStates();
  void publishDesiredStates();
  void readDynamixelState();
  void updateDynamixelState();

  void setDesiredToCurrent();
private:
  bool loopback_mode_;
  bool gazebo_sim_mode_;

  std::vector<int> motor_ids_;
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> desired_positions_;
  std::vector<motor_limits_params> all_motor_limits;
  ros::Time current_time_;
  ros::Time last_time_;
  double dt_;

  ros::NodeHandle n_;
  ros::Publisher joint_states_pub_;
  ros::Publisher desired_states_pub_;
  ros::Publisher dt_pub_;

  ros::Subscriber head_command_sub_;

  std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
  dynamixel_multi_driver::DynamixelMultiDriver *multi_driver_;

  WriteValue *writeValue_;

  void commandCallback(const sensor_msgs::JointStateConstPtr& msg);
  void writePositions();
  bool loadDynamixel();
  void checkLoadDynamixel();

  uint32_t convertRadian2Value(float radian, int MOTOR_INDEX);
  double convertValue2Radian(int32_t value, int MOTOR_INDEX);
};
#endif // THREE_DOF_HEAD_INCLUDE_THREE_DOF_HEAD_HEADMANAGER_H_

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tf/transform_listener.h>

#include<geometry_msgs/Pose.h>

#include <ubot_msgs/BalancerStatus.h>
#include <ubot_msgs/JointPositions.h>
#include <ubot_msgs/SetJointLinearPositions.h>
#include <ubot_msgs/SetStationkeep.h>
#include <ubot_msgs/uBot.h>

#include <ubot_control/GazeTarget.h>
#include <ubot_control/GazeControlFeedback.h>

using namespace std;

enum {
  IDLE,
  START,
  PREEMPT,
  SUCCESS
};

class GazeController
{
private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  string robot_frame_;
  string camera_frame_;
  tf::Transform odometry_;
  double odometry_heading_;

  double current_torso_angle_;
  double current_head_tilt_angle_;
  double current_body_angle_;

  bool stationkeep_enabled_;

  ros::Subscriber current_pose_sub_;
  ros::Subscriber balancer_sub_;
  ros::Subscriber joint_pos_sub_;
  ros::ServiceServer gaze_control_service_;
  ros::ServiceServer gc_feedback_service_;
  ros::ServiceClient joint_linear_move_client_;
  ros::ServiceClient stationkeep_client_;

  ros::Time goal_stamp_;
  bool use_base_;
  bool is_position_target_;
  bool is_heading_angle_target_;
  bool is_head_tilt_angle_target_;
  double start_x_;
  double start_y_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_heading_angle_;
  double target_head_tilt_angle_;
  double torso_angle_error_;
  double base_angle_error_;
  double head_tilt_angle_error_;

  int status_feedback_;

  ubot_msgs::SetJointLinearPositions jlp_srv_;
  ubot_msgs::SetStationkeep sk_srv_;

public:
  GazeController (ros::NodeHandle &n, std::string name) :
    nh_(n)
{
    robot_frame_ = "/base_footprint";
    camera_frame_ = "/kinect_optical_frame";
    odometry_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    odometry_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    odometry_heading_ = 0.0;
    current_torso_angle_ = 0.0;
    current_head_tilt_angle_ = 0.0;
    current_body_angle_ = 0.0;
    stationkeep_enabled_ = false;
    use_base_ = true;
    is_position_target_ = false;
    is_heading_angle_target_ = false;
    is_head_tilt_angle_target_ = false;

    start_x_ = 0.0;
    start_y_ = 0.0;
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    target_heading_angle_ = 0.0;
    target_head_tilt_angle_ = 0.0;

    status_feedback_ = IDLE;

    torso_angle_error_ = 0.0;
    base_angle_error_ = 0.0;
    head_tilt_angle_error_ = 0.0;

    current_pose_sub_ = nh_.subscribe("/uBot/current_pose", 1, &GazeController::currentPoseCallback, this);
    balancer_sub_ = nh_.subscribe("/uBot/balancer_status", 1, &GazeController::balancerStatusCallback, this);
    joint_pos_sub_ = nh_.subscribe("/uBot/joint_positions", 1, &GazeController::jointPositionCallback, this);

    gaze_control_service_ = nh_.advertiseService("/uBot/gaze_control", &GazeController::controlGaze, this);
    gc_feedback_service_ = nh_.advertiseService("/uBot/gaze_control_feedback", &GazeController::provideFeedback, this);

    joint_linear_move_client_ = nh_.serviceClient<ubot_msgs::SetJointLinearPositions>("/uBot/set_joint_linear_move");
    stationkeep_client_ = nh_.serviceClient<ubot_msgs::SetStationkeep>("/uBot/set_stationkeep");
}

  ~GazeController() {

  }

  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    odometry_.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0));
    odometry_heading_ = tf::getYaw(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
  }

  void balancerStatusCallback(const ubot_msgs::BalancerStatus& msg) {
    stationkeep_enabled_ = msg.stationkeep_enabled;
    current_body_angle_ = msg.theta;
  }

  void jointPositionCallback(const ubot_msgs::JointPositionsConstPtr& msg) {
    current_torso_angle_ = msg->positions[TORSO];
    current_head_tilt_angle_ = msg->positions[HEAD_TILT];
  }

  bool controlGaze(ubot_control::GazeTarget::Request &req, ubot_control::GazeTarget::Response &resp) {
    resp.success = false;
    start_x_ = odometry_.getOrigin().getX();
    start_y_ = odometry_.getOrigin().getY();
    use_base_ = req.use_base;
    is_position_target_ = req.position_target;
    is_heading_angle_target_ = req.heading_angle_target;
    is_head_tilt_angle_target_ = req.head_tilt_angle_target;
    target_x_ = req.position_x;
    target_y_ = req.position_y;
    target_z_ = req.position_z;
    target_heading_angle_ = req.heading_angle;
    target_head_tilt_angle_ = req.head_tilt_angle;
    status_feedback_ = START;
    resp.success = true;

    return resp.success;
  }

  bool provideFeedback(ubot_control::GazeControlFeedback::Request &req, ubot_control::GazeControlFeedback::Response &resp) {
    resp.done = false;
    resp.goal_stamp = goal_stamp_;

    if (status_feedback_ == SUCCESS) {
      resp.done = true;
      ROS_INFO("Succeeded target gaze (e_torso: % 4.3f, e_base: % 4.3f, e_head: % 4.3f)", torso_angle_error_, base_angle_error_, head_tilt_angle_error_);
      status_feedback_ = IDLE;
    } else if (status_feedback_ == PREEMPT) {
      resp.done = true;
      ROS_ERROR("Preempted target gaze");
      status_feedback_ = IDLE;
    }

    return true;
  }

  void runOnce() {
    if (!is_position_target_ && !is_heading_angle_target_ && !is_head_tilt_angle_target_) {
      return;
    }

    double desired_heading = odometry_heading_;
    double desired_tilt = current_head_tilt_angle_;

    // Compute desired heading angle for torso and/or base and desired tilt angle for the head
    // based on type of target given (position (x,y,z), heading angle, or head tilt angle)
    if (is_position_target_) {
      tf::Vector3 v_wTr = odometry_.getOrigin();
      tf::Vector3 v_wTo (target_x_, target_y_, target_z_);
      tf::Vector3 v_rTo = v_wTo - v_wTr;
      desired_heading = atan2(v_rTo.getY(), v_rTo.getX());

      tf::StampedTransform transform;
      try {
        listener_.lookupTransform("/world", "/kinect_optical_frame", ros::Time(0), transform);
      }
      catch(tf::TransformException &ex) {
        return;
      }
      tf::Vector3 v_camTo = v_wTo - transform.getOrigin();
      double dist_z = v_camTo.getZ();
      v_camTo.setZ(0.0);
      double dist_xy = v_camTo.length();
      // Angle to target from horizontal viewing direction
      desired_tilt = atan2(dist_z, dist_xy);
      // Account for body tilt angle and the coupled head tilt joints
      desired_tilt = (desired_tilt + current_body_angle_) / 2.0;
    } else {
      if (is_heading_angle_target_) {
        desired_heading = target_heading_angle_;
      }

      if (is_head_tilt_angle_target_) {
        // Account for body tilt angle and the coupled head tilt joints
        desired_tilt = (target_head_tilt_angle_ + current_body_angle_) / 2.0;
        if (fabs(desired_tilt) > 1.1) {
          ROS_WARN("Target head tilt angle % 4.3f is outside of limits!", desired_tilt);
          status_feedback_ = PREEMPT;
          return;
        }
      }
    }

    torso_angle_error_ = desired_heading - current_torso_angle_;
    base_angle_error_ = desired_heading - odometry_heading_;
    head_tilt_angle_error_ = desired_tilt - current_head_tilt_angle_;

    if (fabs(torso_angle_error_) < 0.01 &&
        fabs(base_angle_error_) < 0.01 &&
        fabs(head_tilt_angle_error_) < 0.01) {
      is_position_target_ = false;
      is_heading_angle_target_ = false;
      is_head_tilt_angle_target_ = false;
      status_feedback_ = SUCCESS;
    } else {
      double desired_base_angle = odometry_heading_;
      double desired_head_tilt_angle = desired_tilt;
      double desired_torso_angle = desired_heading - odometry_heading_;
      while (desired_torso_angle > M_PI) {
        desired_torso_angle -= 2.0 * M_PI;
      }
      while (desired_torso_angle < -M_PI) {
        desired_torso_angle += 2.0 * M_PI;
      }

      if (use_base_) {
        if (desired_torso_angle > M_PI/2.0) {
          desired_torso_angle = M_PI/2.0;
        }
        if (desired_torso_angle < -M_PI/2.0) {
          desired_torso_angle = -M_PI/2.0;
        }
        desired_base_angle = desired_heading;

        ROS_INFO("Desired base angle: % 4.3f; desired torso angle: % 4.3f", desired_base_angle, desired_torso_angle);

        // Rotate base
        sk_srv_.request.enable = true;
        sk_srv_.request.use_current_pose = false;
        sk_srv_.request.new_pose.header.stamp = ros::Time::now();
        sk_srv_.request.new_pose.header.frame_id = "/world";
        // Use (x,y) position stored at the beginning of service call to account for SK error accumulation
        sk_srv_.request.new_pose.pose.position.x = start_x_;
        sk_srv_.request.new_pose.pose.position.y = start_y_;
        sk_srv_.request.new_pose.pose.position.z = 0.0;
        tf::Quaternion q = tf::createQuaternionFromYaw(desired_base_angle);
        sk_srv_.request.new_pose.pose.orientation.x = q.getX();
        sk_srv_.request.new_pose.pose.orientation.y = q.getY();
        sk_srv_.request.new_pose.pose.orientation.z = q.getZ();
        sk_srv_.request.new_pose.pose.orientation.w = q.getW();

        if (!stationkeep_client_.call(sk_srv_)) {
          ROS_ERROR("Failed to call Stationkeep service!!");
          return;
        }
      } else {
        if (fabs(desired_torso_angle) > M_PI/2.0) {
          ROS_WARN("Target gaze cannot be reached without using base!");
          status_feedback_ = PREEMPT;
          return;
        }
      }

      // Rotate torso and tilt head
      jlp_srv_.request.stamp = ros::Time::now();
      jlp_srv_.request.numJoints = 12;
      jlp_srv_.request.positions.resize(12);
      jlp_srv_.request.velocities.resize(12);
      jlp_srv_.request.durations.resize(12);
      jlp_srv_.request.positions[TORSO] = desired_torso_angle;
      jlp_srv_.request.positions[HEAD_TILT] = desired_head_tilt_angle;
      jlp_srv_.request.velocities[TORSO] = 0.25;
      jlp_srv_.request.velocities[HEAD_TILT] = 0.3;
      jlp_srv_.request.bitmask = (1 << TORSO) | (1 << HEAD_TILT);

      if (!joint_linear_move_client_.call(jlp_srv_)) {
        ROS_ERROR("Failed to call SetJointLinearMove service!!");
        return;
      }

    }
  }
};

int main(int argc, char** argv) {
  ros::init (argc, argv, "gaze_controller");

  ros::NodeHandle n;
  GazeController gc(n, "just_a_name");

  ros::Rate loop_rate(5.0);

  while (ros::ok()) {
    ros::spinOnce();
    gc.runOnce();
    loop_rate.sleep();
  }

  return 0;
}

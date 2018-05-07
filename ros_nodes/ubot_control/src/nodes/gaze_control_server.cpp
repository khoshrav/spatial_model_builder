#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>
#include <umass_control_msgs/ControlAction.h>
#include <umass_control_msgs/ControlCommand.h>
#include <umass_control_msgs/ControlError.h>
#include <umass_control_msgs/ControlCommand.h>
#include <umass_control/controller/Controller.h>
#include <umass_control/controller/controller_util.h>

#include <ubot_msgs/JointPositions.h>
#include <ubot_msgs/SetJointPositions.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <ubot_msgs/uBot.h>
#include <std_msgs/Int32.h>


#define DEFAULT_SERVO_RATE 100

#define LEFT 0
#define RIGHT 1
#define HEAD 2
#define BASE 3

#define DOF_HEAD 2
#define TRACK_AHEAD 0
#define TRACK_HANDS 1
#define TRACK_POS 2

class GazeController {
protected:
    ros::NodeHandle n_;
    actionlib::SimpleActionServer <umass_control_msgs::ControlAction> server_;

    umass_control_msgs::ControlGoal action_goal_;
    umass_control_msgs::ControlFeedback feedback_;
    umass_control_msgs::ControlResult result_;

    umass_control_msgs::ControlCommand head_cmd_;

    ros::Subscriber joint_pos_sub_;
    ros::Subscriber joint_des_pos_sub_;

    ros::Publisher joint_pos_pub_;
    ros::Publisher goal_marker_pub_;
    ros::Publisher head_error_pub_;
    ros::Publisher state_pub_;

    // get joint limits for both arms
    KDL::JntArray joint_positions_;
    KDL::JntArray desired_joint_positions_;
    KDL::JntArray joint_limit_max_;
    KDL::JntArray joint_limit_min_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    KDL::ChainFkSolverPos_recursive *fk_pos_solver_;

    tf::TransformListener tf_listener_;

    std::string urdf_string_;
    std::string root_name_;
    std::string tip_name_;

    bool is_ubot6_;
    int state_;
    int track_mode_;
    Controller gaze_;
    int nJoints_;

    ros::Publisher marker_pub;

public:

    GazeController(ros::NodeHandle &n, std::string name) :
            n_(n), server_(n_, name, false) {


        // load the kinematic tree from the robot_description urdf
        if (!ros::param::get("/robot_description", urdf_string_)) {
            ROS_WARN("Could not retrieve '/robot_description' parameter. Exiting!");
            exit(0);
        }

        if (!kdl_parser::treeFromString(urdf_string_, kdl_tree_)) {
            ROS_ERROR("Failed to construct kdl tree from URDF string.");
        }

        is_ubot6_ = false;
        if (!ros::param::get("/uBot/ubot6", is_ubot6_)) {
            ROS_WARN("Could not retrieve 'ubot6' parameter, setting to default value of 'false'.");
        }
        ROS_INFO("Loaded KDL parser");
        // get the chains
        root_name_ = "head_base_link"; //"head_middle_link";
        tip_name_ = "kinect_optical_frame"; //"head_middle_link"; //

        ROS_INFO("Getting KDL chain");
        if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
            ROS_ERROR("Could not load kdl chain from '%s' to '%s'", root_name_.c_str(), tip_name_.c_str());
        }
        joint_limit_max_.resize(kdl_chain_.getNrOfJoints());
        joint_limit_min_.resize(kdl_chain_.getNrOfJoints());
        joint_positions_.resize(kdl_chain_.getNrOfJoints());
        desired_joint_positions_.resize(kdl_chain_.getNrOfJoints());
        fk_pos_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);

        nJoints_ = joint_limit_max_.data.size();
        loadJointLimits(joint_limit_min_, joint_limit_max_);

        gaze_.Initialize(kdl_chain_, joint_limit_min_, joint_limit_max_, 19);
        ROS_INFO("Initialized gaze controller");
        // set up subscriptions
        joint_pos_sub_ = n_.subscribe("/uBot/joint_positions", 1, &GazeController::jointPositionCallback, this);
        joint_des_pos_sub_ = n_.subscribe("/uBot/joint_desired_positions", 1,  &GazeController::jointDesiredPositionCallback, this);
        ROS_INFO("Subcribers registered");
        // set up publisher
        joint_pos_pub_ = n_.advertise<ubot_msgs::SetJointPositions>("/uBot/set_joint_positions", 1);
        goal_marker_pub_ = n_.advertise<visualization_msgs::Marker>("/uBot/action_goal_markers", 10);
        head_error_pub_ = n_.advertise<umass_control_msgs::ControlError>("/uBot/control/head_error", 1);

        state_pub_ = n_.advertise<std_msgs::Int32>("/uBot/head_track/state", 1);
        ROS_INFO("Publishers spun up");
        state_ = NODE_NO_REFERENCE;

        // register callback functions
        server_.registerGoalCallback(boost::bind(&GazeController::goalCB, this));
        server_.registerPreemptCallback(boost::bind(&GazeController::preemptCB, this));
        server_.start();
        ROS_INFO("Started gaze server %s", name.c_str());

        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        track_mode_ = 0;
    }

    ~GazeController() {
        // delete fk_pos_solver_[LEFT];
        // delete fk_pos_solver_[RIGHT];
        // delete fk_pos_solver_[HEAD];
        // delete fk_pos_solver_[BASE];
    }

    void loadJointLimits(KDL::JntArray &min_limits, KDL::JntArray &max_limits) {
        urdf::Model model;
        if (!model.initString(urdf_string_)) {
            ROS_ERROR("Failed to get URDF file!");
        }
        ROS_INFO("Loading joint limits from URDF string.");
            boost::shared_ptr<const urdf::Link> link = model.getLink(tip_name_);
            boost::shared_ptr<const urdf::Joint> joint;

            int j = 1 - 1;
            while (link && j > -1) {
                joint = model.getJoint(link->parent_joint->name);
                if (joint->type == urdf::Joint::REVOLUTE) {
                    ROS_INFO("getting bounds for joint: [%s]", joint->name.c_str());
                    max_limits(j) = joint->limits->upper;
                    min_limits(j) = joint->limits->lower;
                    ROS_INFO("Loaded Min [%f]", min_limits(j));
                    ROS_INFO("Loaded Max [%f]", max_limits(j));
                    j--;
                }
                link = model.getLink(link->getParent()->name);
            }

    }

    void jointPositionCallback(const ubot_msgs::JointPositionsConstPtr &msg) {
        // left
        joint_positions_(0) = msg->positions[HEAD_TILT];
        joint_positions_(1) = msg->positions[HEAD_TILT];

        // joint_positions_[BASE](0) = msg->positions[TORSO];

    }

    void jointDesiredPositionCallback(const ubot_msgs::JointPositionsConstPtr &msg) {
        // load the left desired positions
        desired_joint_positions_(0) = msg->positions[HEAD_TILT];
        desired_joint_positions_(1) = msg->positions[HEAD_TILT];

    }

    void visualizeGoals(umass_control_msgs::ControlCommand cmd, std::string name, int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = cmd.reference;
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cmd.endpoint.position.x;
        marker.pose.position.y = cmd.endpoint.position.y;
        marker.pose.position.z = cmd.endpoint.position.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- length, diameter
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        goal_marker_pub_.publish(marker);
    }

    void removeGoalMarkers(umass_control_msgs::ControlCommand cmd, std::string name, int id) {
//    ROS_INFO("Deleting marker %d", id);
        visualization_msgs::Marker marker;
        marker.header.frame_id = cmd.reference;
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = id;
        marker.action = visualization_msgs::Marker::DELETE;
        goal_marker_pub_.publish(marker);
    }

    void goalCB() {
        ROS_INFO("Received new goal.");

        // accept the new goal
        action_goal_ = *server_.acceptNewGoal();
        // set head mode, currently not used
        track_mode_ = action_goal_.head_track_mode;
        if(track_mode_ == TRACK_AHEAD)
        {
          ROS_ERROR_STREAM("THIS IS NOT YET IMPLEMENTED, CANCELLING GOAL");
          result_.controller_state=-1;
          server_.setAborted(result_, "THIS IS NOT YET IMPLEMENTED, CANCELLING GOAL");
          server_.shutdown();
        }
        else if(track_mode_ == TRACK_HANDS)
        {
          ROS_ERROR_STREAM("THIS IS NOT YET IMPLEMENTED, CANCELLING GOAL");
          result_.controller_state=-1;
          server_.setAborted(result_, "THIS IS NOT YET IMPLEMENTED, CANCELLING GOAL");
          server_.shutdown();
        }
        else if(track_mode_ == TRACK_POS)
        {
          // TODO: NEED TO CHECK THAT THIS GOAL IS VALID...ie in front of the robot

          if (action_goal_.reference != "base_footprint")
          {
            // if the goal is not in base_footprint transform it
            geometry_msgs::PointStamped in;
            geometry_msgs::PointStamped out;
            in.point = action_goal_.center_goal.position;
            in.header.frame_id = action_goal_.reference ;
            // in.header.stamp = ros::Time::now() - ros::Duration(0.1);
            tf_listener_.transformPoint("base_footprint", ros::Time(0), in, in.header.frame_id, out);

            head_cmd_.endpoint.position = out.point;
            head_cmd_.reference = "base_footprint";

          }
          else
          {
            head_cmd_.endpoint.position = action_goal_.center_goal.position;
            head_cmd_.reference = action_goal_.reference;
          }
          if(head_cmd_.endpoint.position.x > 0.05)
          {
            ROS_INFO_STREAM("ACCEPTING GOAL");
            visualizeGoals(head_cmd_, "Gaze goals", 0);
            gaze_.SetObjGoals(head_cmd_);
            state_ = NODE_TRANSIENT;
          }
          else
          {
            result_.controller_state=-1;
            ROS_ERROR_STREAM("GOALS ONLY ACCEPTED THAT ARE IN FRONT OF UBOT, CANCELLING GOAL");
            server_.setAborted(result_, "GOALS ONLY ACCEPTED THAT ARE IN FRONT OF UBOT, CANCELLING GOAL");

            server_.shutdown();
          }
        }
    }

    void preemptCB() {
        if (!server_.isActive()) {
            return;
        }

        removeGoalMarkers(head_cmd_, "Gaze goals", 0);
        //removeGoalMarkers(right_cmd_, "Endpoint goals", 1);

        ROS_WARN("Preempted.");

        server_.setPreempted();
    }

    // void set_hands_goal(){
    //     KDL::Frame current_frame_left;
    //     KDL::Frame current_frame_right;
    //     int retl = fk_pos_solver_[LEFT]->JntToCart(joint_positions_[LEFT],
    //                                            current_frame_left);
    //     int retr = fk_pos_solver_[RIGHT]->JntToCart(joint_positions_[RIGHT],
    //                                             current_frame_right);
    //     ROS_INFO("got joints");
    //     if (retl < 0 || retr < 0){
    //         ROS_WARN("FK error");
    //     }
    //     head_cmd_.endpoint.position.x = (current_frame_left.p.x() + current_frame_right.p.x()) / 2.;
    //     head_cmd_.endpoint.position.y = (current_frame_left.p.y() + current_frame_right.p.y()) / 2.;
    //     head_cmd_.endpoint.position.z = (current_frame_left.p.z() + current_frame_right.p.z()) / 2.;
    //
    //     visualizeGoals(head_cmd_, "Gaze goals", 0);
    // }
    //
    // void set_straight_goal(){
    //     KDL::Frame current_frame;
    //     int ret = fk_pos_solver_[LEFT]->JntToCart(joint_positions_[LEFT],
    //                                            current_frame);
    //     if (ret < 0){
    //         ROS_WARN("FK error");
    //     }
    //     KDL::Vector base_gaze(1., 0., 0.1136);
    //     KDL::Vector gaze_ut = current_frame.M * base_gaze;
    //     head_cmd_.endpoint.position.x = gaze_ut(0);
    //     head_cmd_.endpoint.position.y = gaze_ut(1);
    //     head_cmd_.endpoint.position.z = gaze_ut(2);
    //
    // }

    void run() {

        std_msgs::Int32 state_info;
        state_info.data = state_;
        state_pub_.publish(state_info);

        if (!server_.isActive()) {
            return;
        }

        if (state_ == NODE_NO_REFERENCE) {
            // waiting for a goal
        } else if (state_ == NODE_TRANSIENT) {
            // // get new joint positions
            // if (track_mode_ == 0) {
            //     ROS_INFO("pre straight");
            //     set_straight_goal();
            //     ROS_INFO("post straight");
            // }
            // else if (track_mode_ == 1) {
            //     ROS_INFO("pre hand");
            //     set_hands_goal();
            //     ROS_INFO("post hand");
            // }

            KDL::JntArray head_jnt_pos;
            head_jnt_pos.resize(1);
            double head_e_dot;
            head_e_dot = gaze_.ComputeNewJointPositionsWithFeedback(joint_positions_, desired_joint_positions_, head_jnt_pos);
            // publish error information
            // umass_control_msgs::ControlError head_error_msg;
            // head_error_msg.header.stamp = ros::Time::now();
            //
            // head_error_msg.potential = gaze_.GetPotential();
            // head_error_msg.potential_dot = gaze_.GetPotentialDot();
            // head_error_msg.potential_dot_controller = head_e_dot;
            // head_error_pub_.publish(head_error_msg);


            // set up new joint position
            ubot_msgs::SetJointPositions joint_pos;
            joint_pos.stamp = ros::Time::now();
            joint_pos.positions.resize(12);
            joint_pos.numJoints = 12;
            joint_pos.positions[11] = head_jnt_pos(0);
            // joint_pos.positions[HEAD_TILT] = head_jnt_pos(0);

            // TODO: CHECK BITMASK
            joint_pos.bitmask |= 2048;
            // publish new positions
            joint_pos_pub_.publish(joint_pos);

            int headState[1];
            gaze_.GetConvergedBits(headState);

            ROS_INFO("Gaze error remaining: %4.3f", sqrt(gaze_.GetPotential()));

            ROS_INFO_STREAM("GAZE MODE: " << track_mode_ << " CONVERGED " << (headState[0] == NODE_CONVERGED));
            // if controllers are converged, converge
            if (headState[0] == NODE_CONVERGED) {
                state_ = NODE_CONVERGED;
                result_.controller_state = state_;
                server_.setSucceeded(result_);
                ROS_INFO("Controller succeeded!");

                removeGoalMarkers(head_cmd_, "Gaze goals", 0);
            }

        } else if (state_ == NODE_CONVERGED) {

        }
        //get_feedback();
        //server_.publishFeedback(feedback_);

    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "GazeController"); //need to use launch script to run multiple copies
    ros::NodeHandle n;
    GazeController node(n, "GC");

    ros::Rate loop_rate(DEFAULT_SERVO_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        node.run();
        loop_rate.sleep();

    }
    return 0;
}

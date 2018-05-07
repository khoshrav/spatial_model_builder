#!/usr/bin/env python
import roslib
import rospy
import math


from geometry_msgs.msg import Pose, Point, PoseStamped , PointStamped, WrenchStamped
from controller_msgs.srv import ExecuteAction
from umass_control_msgs.msg import ControlAction, ControlGoal
from ubot_control.msg import BracingAction, BracingGoal
from ubot_nav.msg import DriveAction, DriveGoal
from ubot_nav.srv import *

from ubot_msgs.msg import *
from ubot_msgs.srv import *
from ubot_postural_control.control_utils import *
from ubot_spline_controller.msg import *

from tf import TransformListener

import tf
import actionlib
import random
import copy
import tf_conversions.posemath as pm
import PyKDL
import numpy as np
from tactile_properties.srv import Push, PushRequest


class uBotActionManager:
    def __init__(self):
        print 'ubot controller running'
        self.listener = TransformListener()

        rospy.Subscriber("/uBot/joint_positions", JointPositions, self.joint_pos_callback)
        rospy.Subscriber("/uBot/current_pose", PoseStamped, self.current_pose_callback)
        self.pub = rospy.Publisher('/test_pose_recognition', PoseStamped, queue_size=10)
        self.pub_hand = rospy.Publisher('/t', PointStamped, queue_size=10)
        self.pub_hand2 = rospy.Publisher('/t2', PointStamped, queue_size=10)

        self.s = rospy.Service('/umass/execute_action', ExecuteAction, self.handle_execute_action)
        self.place_client = actionlib.SimpleActionClient("BEPC", ControlAction)
        self.squeeze_client = actionlib.SimpleActionClient("BGC", ControlAction)
        self.lift_client = actionlib.SimpleActionClient("BLC", ControlAction)
        # pseudo force controller
        self.grasp_client = actionlib.SimpleActionClient("GraspServer", ControlAction)
        # pseudo force based lift controller
        self.grasplift_client = actionlib.SimpleActionClient("ForcePositionServer", ControlAction)
        self.force_pos_rotation_client = actionlib.SimpleActionClient("ForcePositionRotationServer", ControlAction)
        # bracing controller
        self.bracing_control_client = actionlib.SimpleActionClient("BracingController", BracingAction)

        self.drive_client = actionlib.SimpleActionClient("PathFollowerServer", DriveAction)
        rospy.wait_for_service('/uBot/set_joint_linear_move')
        self.joint_linear_position_client = rospy.ServiceProxy('/uBot/set_joint_linear_move', SetJointLinearPositions)
        self.spline_client = actionlib.SimpleActionClient('/ubot_spline_controller', SplineAction)

        self.add_box_obstacle_client = rospy.ServiceProxy('/add_box_obstacle', addRemoveBoxObstacle)
        self.remove_box_obstacle_client = rospy.ServiceProxy('/remove_box_obstacle', addRemoveBoxObstacle)

        self.home_pos = [0] * 12

        self.home_pos[TORSO] = 0.0

        self.home_pos[R_TILT] = -0.6
        self.home_pos[R_PAN] = -0.4
        self.home_pos[R_TWIST] = -0.1
        self.home_pos[R_ELBOW] = 0.8

        self.home_pos[L_TILT] = 0.6
        self.home_pos[L_PAN] = 0.4
        self.home_pos[L_TWIST] = 0.1
        self.home_pos[L_ELBOW] = 0.8
        self.home_pos[HEAD_TILT] = -0.25

        rospy.sleep(1.0)
        self.home()
        self.set_head(-0.25)

        self.jointPos = list([0] * 12)
        self.push_client = rospy.ServiceProxy('/push', Push)

    def joint_pos_callback(self, jointPosMsg):
        # update local copy of current joint positions whenever new joint
        # positions are published
        # print 'get joint positions'
        self.jointPos = list(jointPosMsg.positions)

    def current_pose_callback(self , poseMsg):
        self.current_pose = poseMsg

    def add_cube_obstacle(self, box_frame):
        print "adding obstacle for cube"

        goal = addRemoveBoxObstacleRequest()
        goal.stamp = rospy.Time.now()

        # feature_frame = pm.fromMsg()
        goal.box_location = box_frame
        response = self.add_box_obstacle_client(goal)

        print "done adding box obstacle"

        return response

        # try:
        #     w_to_base = pm.fromTf(self.listener.lookupTransform('/world', '/base_footprint', rospy.Time(0)))
        #
        #     pose_msg = pm.toMsg(w_to_base * box_frame)
        #     goal.box_location = pose_msg
        #
        #     response = self.add_box_obstacle_client(goal)
        #
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print ("transform not available")

    def remove_cube_obstacle(self, box_frame):
        print "removing obstacle for cube"

        goal = addRemoveBoxObstacleRequest()
        goal.stamp = rospy.Time.now()

        # feature_frame = pm.fromMsg()
        goal.box_location = box_frame
        response = self.remove_box_obstacle_client(goal)

        return response

        # try:
        #     w_to_base = pm.fromTf(self.listener.lookupTransform('/world', '/base_footprint', rospy.Time(0)))
        #
        #     pose_msg = pm.toMsg(w_to_base * box_frame)
        #     goal.box_location = pose_msg
        #
        #     response = self.remove_box_obstacle_client(goal)
        #
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print ("transform not available")

    def whats_up_gesture(self):
        print 'What\'s up?'

        sg = SplineGoal()
        sg.positions = []
        sg.times = []

        upPos = self.jointPos

        sg.positions = sg.positions + upPos
        sg.times = sg.times + [0.0]

#         upPos[TORSO] = 0.0
        upPos[R_TILT] = -0.617
        upPos[L_TILT] = 0.617
        upPos[R_PAN] = -0.35
        upPos[L_PAN] = 0.35
        upPos[R_TWIST] = -0.85
        upPos[L_TWIST] = 0.85
        upPos[R_ELBOW] = 0.25
        upPos[L_ELBOW] = 0.25

        sg.positions = sg.positions + upPos
        sg.times = sg.times + [4.0]

#         upPos[TORSO] = 0.0
#         upPos[R_TILT] = -0.817
#         upPos[L_TILT] = 0.817
#         upPos[R_PAN] = -1.0  # -1.35
#         upPos[L_PAN] = 1.0  # 1.35
#         upPos[R_TWIST] = -1.45
#         upPos[L_TWIST] = 1.45
#         upPos[R_ELBOW] = 0.65
#         upPos[L_ELBOW] = 0.65
#         sg.positions = sg.positions + upPos
#         sg.times = sg.times + [6.0]

#         upPos[TORSO] = 0.0
        upPos[R_TILT] = -0.617
        upPos[L_TILT] = 0.617
        upPos[R_PAN] = -0.35
        upPos[L_PAN] = 0.35
        upPos[R_TWIST] = -0.85
        upPos[L_TWIST] = 0.85
        upPos[R_ELBOW] = 0.25
        upPos[L_ELBOW] = 0.25
        sg.positions = sg.positions + upPos
        sg.times = sg.times + [8.0]

#        upPos[R_TILT] = -0.817
#        upPos[L_TILT] = 0.817
#        upPos[R_PAN] = -1.35
#        upPos[L_PAN] = 1.35
#        upPos[R_TWIST] = -1.45
#        upPos[L_TWIST] = 1.45
#        upPos[R_ELBOW] = 0.65
#        upPos[L_ELBOW] = 0.65
#        sg.positions = sg.positions + upPos
#        sg.times = sg.times + [8.0]

        # back to home
        upPos = list(self.home_pos)
        sg.positions = sg.positions + upPos
        sg.times = sg.times + [10.0]

        self.spline_client.wait_for_server()
        self.spline_client.send_goal(sg)
        self.spline_client.wait_for_result()

    def done_gesture(self):
        print 'I have done it.'

        sg = SplineGoal()
        sg.positions = []
        sg.times = []

        upPos = self.jointPos

        sg.positions = sg.positions + upPos
        sg.times = sg.times + [0.0]


#         upPos[L_TILT] = -4.5
#         upPos[R_TILT] = 4.5
#         upPos[L_PAN] = 1.0
#         upPos[R_PAN] = -1.0
#         upPos[L_TWIST] = -2.0
#         upPos[R_TWIST] = 2.0
        upPos[L_ELBOW] = 0.0
        upPos[R_ELBOW] = 0.0
#         upPos[HEAD_TILT] = 0.35
        sg.positions = sg.positions + upPos
        sg.times = sg.times + [2.0]

#         upPos[L_TILT] = -3.3
#         upPos[R_TILT] = 3.3
#         upPos[L_PAN] = 0.0
#         upPos[R_PAN] = 0.0
#         upPos[L_TWIST] = 0.0
#         upPos[R_TWIST] = 0.0
#         upPos[L_ELBOW] = 0.35
#         upPos[R_ELBOW] = 0.35
#         upPos[HEAD_TILT] = 0.55
#         sg.positions = sg.positions + upPos
#         sg.times = sg.times + [4.0]

        # back to home
        upPos = list(self.home_pos)
        sg.positions = sg.positions + upPos
        sg.times = sg.times + [4.0]

        self.spline_client.wait_for_server()
        self.spline_client.send_goal(sg)
        self.spline_client.wait_for_result()

    def home_spline(self):
        print 'homing'

        sg = SplineGoal()
        sg.positions = []
        sg.times = []

        upPos = self.jointPos

        sg.positions = sg.positions + upPos
        sg.times = sg.times + [0.0]

        # back to home
        upPos = list(self.home_pos)
        sg.positions = sg.positions + upPos
        sg.times = sg.times + [3.0]

        print 'current pos:'
        print self.jointPos
        print 'home pos:'
        print self.home_pos

        self.spline_client.wait_for_server()
        self.spline_client.send_goal(sg)
        self.spline_client.wait_for_result()

    def home(self):
        print 'homing'

        joint_linear_position_request = SetJointLinearPositionsRequest()
        joint_linear_position_request.stamp = rospy.Time.now()
        joint_linear_position_request.numJoints = 12
        joint_linear_position_request.positions = [0]*12
        joint_linear_position_request.velocities = [0]*12
        joint_linear_position_request.durations = [3]*12

        joint_linear_position_request.positions = self.home_pos

        joint_linear_position_request.bitmask = BOTH_ARMS

        response = self.joint_linear_position_client(joint_linear_position_request)
        rospy.sleep(0.1)
        response = self.joint_linear_position_client(joint_linear_position_request)

        rospy.sleep(3.5)

#        self.whats_up_gesture()

    def set_head(self, angle):
        joint_linear_position_request = SetJointLinearPositionsRequest()
        joint_linear_position_request.stamp = rospy.Time.now()
        joint_linear_position_request.numJoints = 12
        joint_linear_position_request.positions = [0]*12
        joint_linear_position_request.velocities = [0]*12
        joint_linear_position_request.durations = [2]*12

        joint_linear_position_request.positions[HEAD_TILT] = angle

        joint_linear_position_request.bitmask = (1 << HEAD_TILT)

        response = self.joint_linear_position_client(joint_linear_position_request)

        rospy.sleep(2.5)

    def handle_execute_action(self, req):
        print 'handle msg'
        if req.action_type == 0:
            print 'going home'
            self.home()

        elif req.action_type == 1:
            print 'execute place action'

            # determine pose of the goals relative to the tag
            feature_frame = pm.fromMsg(req.feature_poses[0])

            left_pose_diff = Pose()
            right_pose_diff = Pose()

            # offsets for the actions in tag frame
            left_pose_diff.position.x = req.parameters[0]
            left_pose_diff.position.y = req.parameters[1]
            left_pose_diff.position.z = req.parameters[2]
            left_pose_diff.orientation.x = 0
            left_pose_diff.orientation.y = 0
            left_pose_diff.orientation.z = 0
            left_pose_diff.orientation.w = 1

            right_pose_diff.position.x = req.parameters[3]
            right_pose_diff.position.y = req.parameters[4]
            right_pose_diff.position.z = req.parameters[5]
            right_pose_diff.orientation.x = 0
            right_pose_diff.orientation.y = 0
            right_pose_diff.orientation.z = 0
            right_pose_diff.orientation.w = 1

            # print left_pose_diff
            l_pose_diff_frame = pm.fromMsg(left_pose_diff)
            r_pose_diff_frame = pm.fromMsg(right_pose_diff)

            left_pose = pm.toMsg(feature_frame * l_pose_diff_frame)
            # print left_pose
            right_pose = pm.toMsg(feature_frame * r_pose_diff_frame)

            goal = ControlGoal()
            goal.reference = 'base_footprint'  # 'world' #

            goal.left_goal.position = left_pose.position
            goal.right_goal.position = right_pose.position

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()
            rospy.sleep(3)
            return 1

        elif req.action_type == 2:
            print 'execute squeeze action'

            goal = ControlGoal()
            goal.magnitude = req.parameters[0]

            self.squeeze_client.wait_for_server()
            self.squeeze_client.send_goal(goal)

            self.squeeze_client.wait_for_result()
            rospy.sleep(0)
            return 1

        elif req.action_type == 3:
            print 'execute lift action'

            # determine pose of the goals relative to the tag
#             feature_frame =  pm.fromMsg(req.feature_poses[0])
#
#             left_pose_diff = Pose()
#             right_pose_diff = Pose()
#
#             # offsets for the actions in tag frame
#             left_pose_diff.position.x = 0
#             left_pose_diff.position.y = 0
#             left_pose_diff.position.z = req.parameters[0]
#             left_pose_diff.orientation.x = 0
#             left_pose_diff.orientation.y = 0
#             left_pose_diff.orientation.z = 0
#             left_pose_diff.orientation.w = 1
#
#             right_pose_diff.position.x = 0
#             right_pose_diff.position.y = 0
#             right_pose_diff.position.z = req.parameters[0]
#             right_pose_diff.orientation.x = 0
#             right_pose_diff.orientation.y = 0
#             right_pose_diff.orientation.z = 0
#             right_pose_diff.orientation.w = 1
#
#             #print left_pose_diff
#             l_pose_diff_frame = pm.fromMsg(left_pose_diff)
#             r_pose_diff_frame = pm.fromMsg(right_pose_diff)
#
#             left_pose = pm.toMsg(feature_frame * l_pose_diff_frame)
#             #print left_pose
#             right_pose = pm.toMsg(feature_frame * r_pose_diff_frame)
#
#
#             print 'left pose ', left_pose

            goal = ControlGoal()
            goal.reference = 'base_footprint'  # 'world' #

            goal.left_goal.position.z = req.parameters[0]
            goal.right_goal.position.z = req.parameters[0]

            goal.magnitude = 0

            self.lift_client.wait_for_server()
            self.lift_client.send_goal(goal)

            self.lift_client.wait_for_result()
            rospy.sleep(1.5)
            return 1

        elif req.action_type == 4:

            print 'execute egocentric place action'

            left_p = Point()
            left_p.x = req.parameters[0]
            left_p.y = req.parameters[1]
            left_p.z = req.parameters[2]

            right_p = Point()
            right_p.x = req.parameters[3]
            right_p.y = req.parameters[4]
            right_p.z = req.parameters[5]

            goal = ControlGoal()

            goal.left_goal.position = left_p
            goal.right_goal.position = right_p
            goal.reference = 'base_footprint'

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result(rospy.Duration.from_sec(0.0))

            self.home()
            rospy.sleep(0)
            return 1

        elif req.action_type == 5:

            print 'execute drive action'

            pose_msgs = []
            # for i in range(len(req.feature_poses)-1, -1, -1):
            for i in range(len(req.feature_poses)-1, 0, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # drive up to box
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y, req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x, req.feature_poses[i].position.y, req.feature_poses[i].position.z))
                pose_msgs.append(pm.toMsg(frame))
                if not self.add_cube_obstacle(pm.toMsg(frame)):
                    print 'Failed to add cube obstacle!'
                    return 0

            # drive pose given in this frame
            feature_frame = pm.fromMsg(req.feature_poses[0])

            drive_pose_diff = Pose()
            drive_pose_diff.position.x = req.parameters[0]
            drive_pose_diff.position.y = req.parameters[1]
            drive_pose_diff.position.z = req.parameters[2]
            drive_pose_diff.orientation.x = req.parameters[3]
            drive_pose_diff.orientation.y = req.parameters[4]
            drive_pose_diff.orientation.z = req.parameters[5]
            drive_pose_diff.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose_diff.orientation.x, 2) + math.pow(drive_pose_diff.orientation.y, 2) + math.pow(drive_pose_diff.orientation.z, 2) + math.pow(drive_pose_diff.orientation.w, 2))
            drive_pose_diff.orientation.x /= mag
            drive_pose_diff.orientation.y /= mag
            drive_pose_diff.orientation.z /= mag
            drive_pose_diff.orientation.w /= mag

            drive_pose_diff_frame = pm.fromMsg(drive_pose_diff)
            temp_frame = feature_frame * drive_pose_diff_frame
            temp_pose = pm.toMsg(temp_frame)

            drive_rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w)
            drive_unit_x = drive_rot.UnitX()
            drive_yaw = math.atan2(drive_unit_x.y(), drive_unit_x.x())
            drive_orient = PyKDL.Rotation.RotZ(drive_yaw)
            drive_frame = PyKDL.Frame(drive_orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y, temp_pose.position.z))
            drive_pose = pm.toMsg(drive_frame)

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            rospy.sleep(3.0)

            for i in range(len(pose_msgs)):
                try:
                    if not self.remove_cube_obstacle(pose_msgs[i]):
                        print 'Failed to remove cube obstacle'
                        return 0
                except NameError:
                    pass

            return 1

        elif req.action_type == 6:

            print 'execute drive backward action'

            drive_pose = Pose()

            drive_pose.position.x = req.parameters[0]
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'  # 'world' #
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose

            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()
            self.set_head(-0.25)
#             rospy.sleep(1)
            return 1

        elif req.action_type == 7:

            print 'execute drop action'

            goal = ControlGoal()
            goal.reference = 'base_footprint'  # 'world'

            goal.left_goal.position.z = -0.4
            goal.right_goal.position.z = -0.4

            goal.magnitude = 0

            self.lift_client.wait_for_server()
            self.lift_client.send_goal(goal)

            self.lift_client.wait_for_result()

            goal = ControlGoal()
            goal.magnitude = -0.08

            self.squeeze_client.wait_for_server()
            self.squeeze_client.send_goal(goal)

            self.squeeze_client.wait_for_result()

            left_p = Point()
            left_p.x = req.parameters[0]
            left_p.y = req.parameters[1]
            left_p.z = req.parameters[2]

            right_p = Point()
            right_p.x = req.parameters[3]
            right_p.y = req.parameters[4]
            right_p.z = req.parameters[5]

            goal = ControlGoal()

            goal.left_goal.position = left_p
            goal.right_goal.position = right_p
            goal.reference = 'base_footprint'

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result(rospy.Duration.from_sec(0.0))

            drive_pose = Pose()

            drive_pose.position.x = -0.15
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'  # 'world' #
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose

            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.home()
            self.drive_client.wait_for_result()

            rospy.sleep(1)
            return 1

        elif req.action_type == 8:

            print 'execute DEBUG DRIVE action'

            drive_pose = Pose()

            drive_pose.position.x = req.parameters[0]
            drive_pose.position.y = req.parameters[1]
            drive_pose.position.z = req.parameters[2]
            drive_pose.orientation.x = req.parameters[3]
            drive_pose.orientation.y = req.parameters[4]
            drive_pose.orientation.z = req.parameters[5]
            drive_pose.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose.orientation.x, 2) + math.pow(drive_pose.orientation.y, 2) + math.pow(drive_pose.orientation.z, 2) + math.pow(drive_pose.orientation.w, 2))
            drive_pose.orientation.x /= mag
            drive_pose.orientation.y /= mag
            drive_pose.orientation.z /= mag
            drive_pose.orientation.w /= mag

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'  # 'world' #
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose

            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()
            self.set_head(-0.49)
            # rospy.sleep(2)
            return 1

        elif req.action_type == 9:
            self.whats_up_gesture()

        elif req.action_type == 10:
            self.done_gesture()

        elif req.action_type == 11:

            print 'execute DEBUG egocentric place action'

            left_p = Point()
            left_p.x = req.parameters[0]
            left_p.y = req.parameters[1]
            left_p.z = req.parameters[2]

            right_p = Point()
            right_p.x = req.parameters[3]
            right_p.y = req.parameters[4]
            right_p.z = req.parameters[5]

            goal = ControlGoal()

            goal.left_goal.position = left_p
            goal.right_goal.position = right_p
            goal.reference = 'base_footprint'

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result(rospy.Duration.from_sec(0.0))

            rospy.sleep(0)
            return 1

        elif req.action_type == 12:
            print 'execute pseudo force grasp action'

            goal = ControlGoal()
            goal.magnitude = req.parameters[0]

            self.grasp_client.wait_for_server()
            self.grasp_client.send_goal(goal)

            self.grasp_client.wait_for_result()
            rospy.sleep(0)
            return 1

        elif req.action_type == 13:
            print 'execute pseudo force grasp lift action'

            # determine pose of the goals relative to the tag
            # feature_frame =  pm.fromMsg(req.feature_poses[0])

            center_p = Point()
            center_p.x = req.parameters[1]
            center_p.y = req.parameters[2]
            center_p.z = req.parameters[3]

            print "Lifting to ", center_p.x, ", ", center_p.y, ", ", center_p.z

            goal = ControlGoal()
            goal.reference = 'upper_trunk'  # 'world' #

            goal.center_goal.position = center_p

            goal.magnitude = req.parameters[0]

            self.grasplift_client.wait_for_server()
            self.grasplift_client.send_goal(goal)

            self.grasplift_client.wait_for_result()
            rospy.sleep(1.5)
            return 1

        elif req.action_type == 14:
            # compound flip action including placing hands, grasping, lifting, and placing of the object
            print 'execute compound flip action (consists of 4 fine grained actions)'

            # place hands

            # determine pose of the goals relative to the tag
            feature_frame = pm.fromMsg(req.feature_poses[0])

            left_pose_diff = Pose()
            right_pose_diff = Pose()

            # offsets for the actions in tag frame
            left_pose_diff.position.x = req.parameters[0]
            left_pose_diff.position.y = req.parameters[1]
            left_pose_diff.position.z = req.parameters[2]
            left_pose_diff.orientation.x = 0
            left_pose_diff.orientation.y = 0
            left_pose_diff.orientation.z = 0
            left_pose_diff.orientation.w = 1

            right_pose_diff.position.x = req.parameters[3]
            right_pose_diff.position.y = req.parameters[4]
            right_pose_diff.position.z = req.parameters[5]
            right_pose_diff.orientation.x = 0
            right_pose_diff.orientation.y = 0
            right_pose_diff.orientation.z = 0
            right_pose_diff.orientation.w = 1

            # print left_pose_diff
            l_pose_diff_frame = pm.fromMsg(left_pose_diff)
            r_pose_diff_frame = pm.fromMsg(right_pose_diff)

            left_pose = pm.toMsg(feature_frame * l_pose_diff_frame)
            # print left_pose
            right_pose = pm.toMsg(feature_frame * r_pose_diff_frame)

            goal = ControlGoal()
            goal.reference = 'base_footprint'  # 'world' #

            goal.left_goal.position = left_pose.position
            goal.right_goal.position = right_pose.position

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()
            rospy.sleep(3)

            # grasp

            goal = ControlGoal()
            goal.magnitude = req.parameters[6]

            self.grasp_client.wait_for_server()
            self.grasp_client.send_goal(goal)

            self.grasp_client.wait_for_result()
            rospy.sleep(0)

            # lift

            center_p = Point()
            center_p.x = req.parameters[7]
            center_p.y = req.parameters[8]
            center_p.z = req.parameters[9]

            print "Lifting to ", center_p.x, ", ", center_p.y, ", ", center_p.z

            goal = ControlGoal()
            goal.reference = 'upper_trunk'  # 'world' #

            goal.center_goal.position = center_p

            goal.magnitude = req.parameters[6]

            self.grasplift_client.wait_for_server()
            self.grasplift_client.send_goal(goal)

            self.grasplift_client.wait_for_result()
            rospy.sleep(1.5)

            # place

            goal = ControlGoal()
            goal.reference = 'base_footprint'  # 'world'

            goal.left_goal.position.z = -0.4
            goal.right_goal.position.z = -0.4

            goal.magnitude = 0

            self.lift_client.wait_for_server()
            self.lift_client.send_goal(goal)

            self.lift_client.wait_for_result()

            goal = ControlGoal()
            goal.magnitude = -0.08

            self.squeeze_client.wait_for_server()
            self.squeeze_client.send_goal(goal)

            self.squeeze_client.wait_for_result()

            left_p = Point()
            left_p.x = req.parameters[10]
            left_p.y = req.parameters[11]
            left_p.z = req.parameters[12]

            right_p = Point()
            right_p.x = req.parameters[13]
            right_p.y = req.parameters[14]
            right_p.z = req.parameters[15]

            goal = ControlGoal()

            goal.left_goal.position = left_p
            goal.right_goal.position = right_p
            goal.reference = 'base_footprint'

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result(rospy.Duration.from_sec(0.0))

            drive_pose = Pose()

            drive_pose.position.x = -0.15
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'  # 'world' #
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose

            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.home()
            self.drive_client.wait_for_result()

            rospy.sleep(1)

            return 1

        # Orbit Action
        # feature_poses:
        #   0 - rotation centroid
        # parameters:
        #   0 - orbit amount in radians
        #   1 - orbit radius
        elif req.action_type == 15:

            box = req.feature_poses[0].position
            pose = self.current_pose.pose.position
            distance_to_start = np.sqrt(math.pow(box.x - pose.x, 2) + math.pow(box.y - pose.y, 2)) - req.parameters[1]

            # Find drive location at certain radius
            # First you must project robot current pose to correct radius
            # http://stackoverflow.com/questions/1800138/given-a-start-and-end-point-and-a-distance-calculate-a-point-along-a-line

            vx = box.x - pose.x
            vy = box.y - pose.y

            mag = np.sqrt(vx * vx + vy * vy)

            vx /= mag
            vy /= mag

            robot_projected_x = box.x - vx * (req.parameters[1])
            robot_projected_y = box.y - vy * (req.parameters[1])

            # you then rotate that new point the correct amount
            # rotating a point http://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
            s = np.sin(req.parameters[0])
            c = np.cos(req.parameters[0])

            x = robot_projected_x - box.x
            y = robot_projected_y - box.y

            rotated_x = x * c - y * s
            rotated_y = x * s + y * c

            rotated_x += box.x
            rotated_y += box.y

            print 'execute orbit action'

            drive_pose = Pose()

            drive_pose.position.x = rotated_x
            drive_pose.position.y = rotated_y
            drive_pose.position.z = 0.0
            print drive_pose

            o = [0 for row in range(4)]
            o[0] = self.current_pose.pose.orientation.x
            o[1] = self.current_pose.pose.orientation.y
            o[2] = self.current_pose.pose.orientation.z
            o[3] = self.current_pose.pose.orientation.w

            # goal orientation should be facing the box
            # euler = tf.transformations.euler_from_quaternion(o)
            # yaw = euler[2] + req.parameters[0] # add in amount of desired rotation
            yaw = math.atan2(box.y - rotated_y, box.x  - rotated_x )
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)  # todo quaterion from yaw in tf in python?

            drive_pose.orientation.x = quaternion[0]
            drive_pose.orientation.y = quaternion[1]
            drive_pose.orientation.z = quaternion[2]
            drive_pose.orientation.w = quaternion[3]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose.orientation.x, 2) + math.pow(drive_pose.orientation.y, 2) + math.pow(drive_pose.orientation.z, 2) + math.pow(drive_pose.orientation.w, 2))
            drive_pose.orientation.x /= mag
            drive_pose.orientation.y /= mag
            drive_pose.orientation.z /= mag
            drive_pose.orientation.w /= mag

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose

            obstacle_poses = []
            for i in range(len(req.feature_poses)):
                temp_pose = req.feature_poses[i]
                rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y, temp_pose.position.z))
                obstacle_poses.append(pm.toMsg(frame))

                self.add_cube_obstacle(obstacle_poses[i])

            drive_pose_pub = PoseStamped()
            drive_pose_pub.pose = drive_pose
            drive_pose_pub.header.frame_id = 'world'
            drive_pose_pub.header.stamp = rospy.Time.now()

            self.pub.publish(drive_pose_pub)
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()
            rospy.sleep(3.0)

            for i in range(len(obstacle_poses)):
                self.remove_cube_obstacle(obstacle_poses[i])

            return 1

        # Push Action
        # feature_poses:
        #   0      - the object frame to apply the lift action on
        #   1 to n - additional object frames used for obstacle server
        # parameters
        #   0   - x position for drive goal (object frame)
        #   1   - y position for drive goal (object frame)
        #   2   - z position for drive goal (object frame)
        #   3   - x orientation for drive goal (object frame)
        #   4   - y orientation for drive goal (object frame)
        #   5   - z orientation for drive goal (object frame)
        #   6   - w orientation for drive goal (object frame)
        #   7   - x position for push start (object frame)
        #   8   - y position for push start (object frame)
        #   9   - z position for push start (object frame)
        #   10  - hand (left or right, 0,1)
        #   11  - push distance
        #   12  - push vector x
        #   13  - push vector y
        #   14  - push vector z
        # -----------------
        # return success
        elif req.action_type == 16:
            if len(req.parameters) != 15:
                rospy.logerror('Invalid number of params, returning!')
                return False

            print 'push action started!!'
            saved_start_pose = self.current_pose.pose
            # Params
            hand = req.parameters[10]
            push_distance = req.parameters[11]

            pose_msgs = []
            for i in range(len(req.feature_poses)-1, -1, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # add cube obstacle
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y, req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x, req.feature_poses[i].position.y, req.feature_poses[i].position.z))
                pose_msgs.append(pm.toMsg(frame))
                self.add_cube_obstacle(pm.toMsg(frame))

            # drive up to box
            drive_pose_diff = Pose()
            drive_pose_diff.position.x = req.parameters[0]
            drive_pose_diff.position.y = req.parameters[1]
            drive_pose_diff.position.z = req.parameters[2]
            drive_pose_diff.orientation.x = req.parameters[3]
            drive_pose_diff.orientation.y = req.parameters[4]
            drive_pose_diff.orientation.z = req.parameters[5]
            drive_pose_diff.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose_diff.orientation.x, 2) + math.pow(drive_pose_diff.orientation.y, 2) + math.pow(drive_pose_diff.orientation.z, 2) + math.pow(drive_pose_diff.orientation.w, 2))
            drive_pose_diff.orientation.x /= mag
            drive_pose_diff.orientation.y /= mag
            drive_pose_diff.orientation.z /= mag
            drive_pose_diff.orientation.w /= mag

            drive_pose_diff_frame = pm.fromMsg(drive_pose_diff)
            temp_frame = feature_frame * drive_pose_diff_frame
            temp_pose = pm.toMsg(temp_frame)

            drive_rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w)
            drive_unit_x = drive_rot.UnitX()
            drive_yaw = math.atan2(drive_unit_x.y(), drive_unit_x.x())
            drive_orient = PyKDL.Rotation.RotZ(drive_yaw)
            drive_frame = PyKDL.Frame(drive_orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y, temp_pose.position.z))
            drive_pose = pm.toMsg(drive_frame)

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            # remove cube obstacle
            for i in range(len(pose_msgs)):
                try:
                    self.remove_cube_obstacle(pose_msgs[i])
                except NameError:
                    pass

            # let the body stabilize before moving
            rospy.sleep(3)

            # place hands before we do the reach & push
            rospy.loginfo('Pre-posturing arm!')
            req2 = SetJointLinearPositionsRequest()
            req2.stamp = rospy.Time.now()

            # zero out stuffs
            req2.positions = [0 for _ in range(12)]
            req2.velocities = [0 for _ in range(12)]
            req2.durations = [5 for _ in range(12)]

            req2.positions[3] = 0.15
            req2.positions[4] = -0.15
            req2.positions[5] = 0.0
            req2.positions[6] = 0.0
            req2.positions[7] = 0.85
            req2.positions[8] = -0.85
            req2.positions[9] = -0.3
            req2.positions[10] = -0.3

            req2.positions[11] = -0.5
            req2.numJoints = 12
            if hand == 0:
                req2.bitmask = LEFT_ARM | (1 << HEAD_TILT)  # 3408
            else:
                req2.bitmask = RIGHT_ARM | (1 << HEAD_TILT)  # 2728

            # print 'Bitmask: ', req2.bitmask

            if not self.joint_linear_position_client(req2):
                rospy.logwarn('FAILED TO CALL SET LINEAR POSITION SERVICE!')
            else:
                rospy.sleep(8)

            # reach goal
            reach_pose_diff = Pose()

            reach_pose_diff.position.x = req.parameters[7]
            reach_pose_diff.position.y = req.parameters[8]
            reach_pose_diff.position.z = req.parameters[9]
            reach_pose_diff.orientation.x = 0
            reach_pose_diff.orientation.y = 0
            reach_pose_diff.orientation.z = 0
            reach_pose_diff.orientation.w = 1

            reach_pose_diff_frame = pm.fromMsg(reach_pose_diff)

            reach = pm.toMsg(feature_frame * reach_pose_diff_frame)  # pm.toMsg(base_T_w * feature_frame * reach_pose_diff_frame)

            goal = ControlGoal()
            goal.reference = 'world'  # 'base_footprint'

            if hand == 0:
                goal.ignore_left = False
                goal.ignore_right = True
                goal.left_goal.position = reach.position
                # there seems to be a 3(ish) cm offset on the real robot
                # goal.left_goal.position.y = goal.left_goal.position.y - 0.03

            # Right Hand
            if hand == 1:
                goal.ignore_right = False
                goal.ignore_left = True
                goal.right_goal.position = reach.position

            # TODO: this should eventually be replaced with a call to reachtouch
            print 'Reaching!'
            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()
            rospy.sleep(3)

            # find out where we touched
            touchedPoint = PointStamped()
            hand_frame = 'right_hand'
            if hand == 0:
                hand_frame = 'left_hand'
            touchedPoint.header.frame_id = hand_frame
            touchedPoint.point.x = 0
            touchedPoint.point.y = 0
            touchedPoint.point.z = 0

            touched = self.listener.transformPoint('world',touchedPoint)
            # this can be read in later as a param, will need to rotate vector from object to base_footprint
            # (forces are viewed as what is applied to the robot)
            push_vector = WrenchStamped()
            push_vector.header.frame_id = 'base_footprint'
            push_vector.wrench.force.x = -1.0
            push_vector.wrench.force.y = 0.0
            push_vector.wrench.force.z = 0.0

            push_goal = PushRequest()
            push_goal.touched = touched.point
            push_goal.distance = push_distance
            push_goal.use_hand = hand_frame
            push_goal.push_vector = push_vector
            print 'Pushing!'
            rospy.wait_for_service('push')
            self.push_client(push_goal)
            rospy.loginfo('Push completed!')
            rospy.sleep(1)

            # drive back and home
            drive_pose = Pose()
            drive_pose.position.x = -0.35
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1

            # self.home()
            self.set_head(-0.3)
            # rospy.sleep(3.0)
            self.home()

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            # self.home()
            # self.set_head(-0.3)
            # rospy.sleep(1.5)
            # self.home()
            self.drive_client.wait_for_result()
            rospy.sleep(3.0)

            return True

        # Lift Action
        # feature_poses:
        #   0      - the object frame to apply the lift action on
        #   1 to n - additional object frames used for obstacle server
        # parameters:
        #   0  - x position for drive goal (object frame)
        #   1  - y position for drive goal (object frame)
        #   2  - z position for drive goal (object frame)
        #   3  - x orientation for drive goal (object frame)
        #   4  - y orientation for drive goal (object frame)
        #   5  - z orientation for drive goal (object frame)
        #   6  - w orientation for drive goal (object frame)
        #   7  - x position for left hand pre-grasp (object frame)
        #   8  - y position for left hand pre-grasp (object frame)
        #   9  - z position for left hand pre-grasp (object frame)
        #   10 - x position for right hand pre-grasp (object frame)
        #   11 - y position for right hand pre-grasp (object frame)
        #   12 - z position for right hand pre-grasp (object frame)
        #   13 - grasp magnitude
        #   14 - x position for lift (upper trunk frame)
        #   15 - y position for lift (upper trunk frame)
        #   16 - z position for lift (upper trunk frame)
        #   17 - x position for left hand retract post put down (base_footprint frame)
        #   18 - y position for left hand retract post put down (base_footprint frame)
        #   19 - z position for left hand retract post put down (base_footprint frame)
        #   20 - x position for right hand retract post put down (base_footprint frame)
        #   21 - y position for right hand retract post put down (base_footprint frame)
        #   22 - z position for right hand retract post put down (base_footprint frame)
        elif req.action_type == 17:
            print 'execute lift action'
            print 'params:'
            print req.parameters

            pose_msgs = []
            for i in range(len(req.feature_poses)-1, -1, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # add cube obstacle
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y, req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x, req.feature_poses[i].position.y, req.feature_poses[i].position.z))
                pose_msgs.append(pm.toMsg(frame))
                self.add_cube_obstacle(pm.toMsg(frame))

            # drive up to box
            drive_pose_diff = Pose()
            drive_pose_diff.position.x = req.parameters[0]
            drive_pose_diff.position.y = req.parameters[1]
            drive_pose_diff.position.z = req.parameters[2]
            drive_pose_diff.orientation.x = req.parameters[3]
            drive_pose_diff.orientation.y = req.parameters[4]
            drive_pose_diff.orientation.z = req.parameters[5]
            drive_pose_diff.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose_diff.orientation.x, 2) + math.pow(drive_pose_diff.orientation.y, 2) + math.pow(drive_pose_diff.orientation.z, 2) + math.pow(drive_pose_diff.orientation.w, 2))
            drive_pose_diff.orientation.x /= mag
            drive_pose_diff.orientation.y /= mag
            drive_pose_diff.orientation.z /= mag
            drive_pose_diff.orientation.w /= mag

            drive_pose_diff_frame = pm.fromMsg(drive_pose_diff)
            temp_frame = feature_frame * drive_pose_diff_frame
            temp_pose = pm.toMsg(temp_frame)

            drive_rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w)
            drive_unit_x = drive_rot.UnitX()
            drive_yaw = math.atan2(drive_unit_x.y(), drive_unit_x.x())
            drive_orient = PyKDL.Rotation.RotZ(drive_yaw)
            drive_frame = PyKDL.Frame(drive_orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y, temp_pose.position.z))
            drive_pose = pm.toMsg(drive_frame)

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            for i in range(len(pose_msgs)):
                try:
                    self.remove_cube_obstacle(pose_msgs[i])
                except NameError:
                    pass

            self.set_head(-0.45)

            # place hands
            left_pose_diff = Pose()
            right_pose_diff = Pose()

            left_pose_diff.position.x = req.parameters[7]
            left_pose_diff.position.y = req.parameters[8]
            left_pose_diff.position.z = req.parameters[9]
            left_pose_diff.orientation.x = 0
            left_pose_diff.orientation.y = 0
            left_pose_diff.orientation.z = 0
            left_pose_diff.orientation.w = 1

            right_pose_diff.position.x = req.parameters[10]
            right_pose_diff.position.y = req.parameters[11]
            right_pose_diff.position.z = req.parameters[12]
            right_pose_diff.orientation.x = 0
            right_pose_diff.orientation.y = 0
            right_pose_diff.orientation.z = 0
            right_pose_diff.orientation.w = 1

            try:
                base_T_w = pm.fromTf(self.listener.lookupTransform('/base_footprint', '/world', rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'transform not available'

            left_pose_diff_frame = pm.fromMsg(left_pose_diff)
            right_pose_diff_frame = pm.fromMsg(right_pose_diff)

            left_pose = pm.toMsg(base_T_w * feature_frame * left_pose_diff_frame)
            right_pose = pm.toMsg(base_T_w * feature_frame * right_pose_diff_frame)

            # TODO: this is a fix to deal with bad pose estimates for objects with very few features
            # We use the average grasp height (z direction) as we are dealing with boxes on flat surfaces
            average_z = (left_pose.position.z + right_pose.position.z) / 2.0
            left_pose.position.z = average_z
            right_pose.position.z = average_z

            goal = ControlGoal()
            goal.reference = 'base_footprint'

            goal.left_goal.position = left_pose.position
            goal.right_goal.position = right_pose.position

            print 'left_goal:'
            print left_pose.position
            print 'right_goal:'
            print right_pose.position

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()
            rospy.sleep(1.0)

            print 'finished place'

            # grasp
            goal = ControlGoal()
            goal.magnitude = req.parameters[13]
            self.grasp_client.wait_for_server()
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()
            rospy.sleep(0)

            print 'finished grasp'

            # lift
            center_p = Point()
            center_p.x = req.parameters[14]
            center_p.y = req.parameters[15]
            center_p.z = req.parameters[16]

            goal = ControlGoal()
            goal.reference = 'upper_trunk'
            goal.center_goal.position = center_p
            goal.magnitude = 0.01  # req.parameters[13]
            self.grasplift_client.wait_for_server()
            self.grasplift_client.send_goal(goal)
            self.grasplift_client.wait_for_result()
            rospy.sleep(1.5)

            print 'finished lift'

            # place
            # goal = ControlGoal()
            # goal.reference = 'base_footprint' #hardcode to upper_trunk inside controller
            # goal.left_goal.position.z = -0.4
            # goal.right_goal.position.z = -0.4
            # goal.magnitude = 0
            # self.lift_client.wait_for_server()
            # self.lift_client.send_goal(goal)
            # self.lift_client.wait_for_result()

            goal = ControlGoal()
            goal.reference = 'upper_trunk'
            center_p.z = -0.34
            goal.center_goal.position = center_p
            goal.magnitude = 0.01  # req.parameters[13]
            self.grasplift_client.wait_for_server()
            self.grasplift_client.send_goal(goal)
            self.grasplift_client.wait_for_result()

            print 'finished put down'

            goal = ControlGoal()
            goal.magnitude = -0.12
            self.squeeze_client.wait_for_server()
            self.squeeze_client.send_goal(goal)
            self.squeeze_client.wait_for_result()

            print 'finished release'

            left_p = Point()
            left_p.x = req.parameters[17]
            left_p.y = req.parameters[18]
            left_p.z = req.parameters[19]

            right_p = Point()
            right_p.x = req.parameters[20]
            right_p.y = req.parameters[21]
            right_p.z = req.parameters[22]

            goal = ControlGoal()
            goal.left_goal.position = left_p
            goal.right_goal.position = right_p
            goal.reference = 'base_footprint'

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()

            print 'finished moving hands away'

            # drive back and home
            drive_pose = Pose()
            drive_pose.position.x = -0.35
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1

            # self.home()
            self.set_head(-0.3)
            # rospy.sleep(3.0)
            self.home()

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            # self.home()
            # self.set_head(-0.3)
            # rospy.sleep(1.5)
            # self.home()
            self.drive_client.wait_for_result()
            rospy.sleep(3.0)

            print 'finished driving back and homing'

            return 1

        # Pickup Action
        # feature_poses:
        #   0      - the object frame to apply the pickup action on
        #   1 to n - additional object frames used for obstacle server
        # parameters:
        #   0  - x position for drive goal (object frame)
        #   1  - y position for drive goal (object frame)
        #   2  - z position for drive goal (object frame)
        #   3  - x orientation for drive goal (object frame)
        #   4  - y orientation for drive goal (object frame)
        #   5  - z orientation for drive goal (object frame)
        #   6  - w orientation for drive goal (object frame)
        #   7  - x position for left hand pre-grasp (object frame)
        #   8  - y position for left hand pre-grasp (object frame)
        #   9  - z position for left hand pre-grasp (object frame)
        #   10 - x position for right hand pre-grasp (object frame)
        #   11 - y position for right hand pre-grasp (object frame)
        #   12 - z position for right hand pre-grasp (object frame)
        #   13 - grasp magnitude

        #   14 - x position for hand center (upper trunk frame)
        #   15 - y position for hand center (upper trunk frame)
        #   16 - z position for hand center (upper trunk frame)
        #   17 - x position for left hand retract post put down (base_footprint frame)
        #   18 - y position for left hand retract post put down (base_footprint frame)
        #   19 - z position for left hand retract post put down (base_footprint frame)
        #   20 - x position for right hand retract post put down (base_footprint frame)
        #   21 - y position for right hand retract post put down (base_footprint frame)
        #   22 - z position for right hand retract post put down (base_footprint frame)
        elif req.action_type == 18:
            print 'execute pickup action'
            print 'params:'
            print req.parameters

            pose_msgs = []
            for i in range(len(req.feature_poses)-1, -1, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # add cube obstacle
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y,
                                                req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x,
                                                         req.feature_poses[i].position.y,
                                                         req.feature_poses[i].position.z))
                pose_msgs.append(pm.toMsg(frame))
                self.add_cube_obstacle(pm.toMsg(frame))

            # drive up to box
            drive_pose_diff = Pose()
            drive_pose_diff.position.x = req.parameters[0]
            drive_pose_diff.position.y = req.parameters[1]
            drive_pose_diff.position.z = req.parameters[2]
            drive_pose_diff.orientation.x = req.parameters[3]
            drive_pose_diff.orientation.y = req.parameters[4]
            drive_pose_diff.orientation.z = req.parameters[5]
            drive_pose_diff.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose_diff.orientation.x, 2) + math.pow(drive_pose_diff.orientation.y, 2) +
                            math.pow(drive_pose_diff.orientation.z, 2) + math.pow(drive_pose_diff.orientation.w, 2))
            drive_pose_diff.orientation.x /= mag
            drive_pose_diff.orientation.y /= mag
            drive_pose_diff.orientation.z /= mag
            drive_pose_diff.orientation.w /= mag

            drive_pose_diff_frame = pm.fromMsg(drive_pose_diff)
            temp_frame = feature_frame * drive_pose_diff_frame
            temp_pose = pm.toMsg(temp_frame)

            drive_rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y,
                                                  temp_pose.orientation.z, temp_pose.orientation.w)
            drive_unit_x = drive_rot.UnitX()
            drive_yaw = math.atan2(drive_unit_x.y(), drive_unit_x.x())
            drive_orient = PyKDL.Rotation.RotZ(drive_yaw)
            drive_frame = PyKDL.Frame(drive_orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y,
                                                                 temp_pose.position.z))
            drive_pose = pm.toMsg(drive_frame)

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            for i in range(len(pose_msgs)):
                try:
                    self.remove_cube_obstacle(pose_msgs[i])
                except NameError:
                    pass

            self.set_head(-0.45)

            # place hands
            left_pose_diff = Pose()
            right_pose_diff = Pose()

            left_pose_diff.position.x = req.parameters[7]
            left_pose_diff.position.y = req.parameters[8]
            left_pose_diff.position.z = req.parameters[9]
            left_pose_diff.orientation.x = 0
            left_pose_diff.orientation.y = 0
            left_pose_diff.orientation.z = 0
            left_pose_diff.orientation.w = 1

            right_pose_diff.position.x = req.parameters[10]
            right_pose_diff.position.y = req.parameters[11]
            right_pose_diff.position.z = req.parameters[12]
            right_pose_diff.orientation.x = 0
            right_pose_diff.orientation.y = 0
            right_pose_diff.orientation.z = 0
            right_pose_diff.orientation.w = 1

            try:
                base_T_w = pm.fromTf(self.listener.lookupTransform('/base_footprint', '/world', rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'transform not available'

            left_pose_diff_frame = pm.fromMsg(left_pose_diff)
            right_pose_diff_frame = pm.fromMsg(right_pose_diff)

            # print 'left_pos_diff:'
            # print left_pose_diff.position
            #
            # print 'right_pos_diff:'
            # print right_pose_diff.position

            left_pose = pm.toMsg(base_T_w * feature_frame * left_pose_diff_frame)
            right_pose = pm.toMsg(base_T_w * feature_frame * right_pose_diff_frame)

            # TODO: this is a fix to deal with bad pose estimates for objects with very few features
            # We use the average grasp height (z direction) as we are dealing with boxes on flat surfaces
            average_z = (left_pose.position.z + right_pose.position.z) / 2.0
            left_pose.position.z = average_z
            right_pose.position.z = average_z

            goal = ControlGoal()
            goal.reference = 'base_footprint'

            goal.left_goal.position = left_pose.position
            goal.right_goal.position = right_pose.position

            # print 'left_goal:'
            # print left_pose.position
            #
            # print 'left actual'
            #
            # print 'right_goal:'
            # print right_pose.position

            self.place_client.wait_for_server()
            self.place_client.send_goal(goal)
            self.place_client.wait_for_result()
            rospy.sleep(1.0)

            # print 'left goal'
            # print goal.left_goal.position
            # print 'right goal'
            # print goal.right_goal.position

            # try:
            #     base_T_left_hand = pm.fromTf(self.listener.lookupTransform('/base_footprint', '/left_hand', rospy.Time(0)))
            #     base_T_right_hand = pm.fromTf(self.listener.lookupTransform('/base_footprint', '/right_hand', rospy.Time(0)))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     print 'transform not available'
            #
            # print 'left actual'
            # print base_T_left_hand.p
            # print 'right actual'
            # print base_T_right_hand.p


            print 'finished place'

            # grasp
            goal = ControlGoal()
            goal.magnitude = req.parameters[13]
            self.grasp_client.wait_for_server()
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()
            rospy.sleep(0)

            rospy.sleep(3)



            print 'finished grasp'

            # lift
            center_p = Point()
            center_p.x = req.parameters[14]
            center_p.y = req.parameters[15]
            center_p.z = req.parameters[16]

            goal = ControlGoal()
            goal.reference = 'upper_trunk'
            goal.center_goal.position = center_p
            goal.magnitude = 0.01  # req.parameters[13]
            self.grasplift_client.wait_for_server()
            self.grasplift_client.send_goal(goal)
            self.grasplift_client.wait_for_result()
            rospy.sleep(1.5)

            print 'finished lift'

            center_p_align = Point()
            center_p_align.x = 0.35  # req.parameters[17]
            center_p_align.y = 0.0  # req.parameters[18]
            center_p_align.z = -0.2  # req.parameters[19]

            align_goal = ControlGoal()
            align_goal.reference = 'upper_trunk'
            align_goal.center_goal.position = center_p_align
            align_goal.left_goal.orientation.x = 0.0
            align_goal.left_goal.orientation.y = 1.0
            align_goal.left_goal.orientation.z = 0.0

            align_goal.right_goal.orientation.x = -align_goal.left_goal.orientation.x
            align_goal.right_goal.orientation.y = -align_goal.left_goal.orientation.y
            align_goal.right_goal.orientation.z = -align_goal.left_goal.orientation.z
            align_goal.magnitude = 0.006

            self.force_pos_rotation_client.wait_for_server()
            self.force_pos_rotation_client.send_goal(align_goal)
            self.force_pos_rotation_client.wait_for_result()

            print 'finished aligning box in front of robot for transport'

            #
            # #
            # # # place
            # # # goal = ControlGoal()
            # # # goal.reference = 'base_footprint' #hardcode to upper_trunk inside controller
            # # # goal.left_goal.position.z = -0.4
            # # # goal.right_goal.position.z = -0.4
            # # # goal.magnitude = 0
            # # # self.lift_client.wait_for_server()
            # # # self.lift_client.send_goal(goal)
            # # # self.lift_client.wait_for_result()
            # #
            # # goal = ControlGoal()
            # # goal.reference = 'upper_trunk'
            # # center_p.z = -0.4
            # # goal.center_goal.position = center_p
            # # goal.magnitude = 0.01  # req.parameters[13]
            # # self.grasplift_client.wait_for_server()
            # # self.grasplift_client.send_goal(goal)
            # # self.grasplift_client.wait_for_result()
            # #
            # # print 'finished put down'
            # #
            # goal = ControlGoal()
            # goal.magnitude = -0.12
            # self.squeeze_client.wait_for_server()
            # self.squeeze_client.send_goal(goal)
            # self.squeeze_client.wait_for_result()
            #
            # print 'finished release'
            # #
            # # left_p = Point()
            # # left_p.x = req.parameters[17]
            # # left_p.y = req.parameters[18]
            # # left_p.z = req.parameters[19]
            # #
            # # right_p = Point()
            # # right_p.x = req.parameters[20]
            # # right_p.y = req.parameters[21]
            # # right_p.z = req.parameters[22]
            # #
            # # goal = ControlGoal()
            # # goal.left_goal.position = left_p
            # # goal.right_goal.position = right_p
            # # goal.reference = 'base_footprint'
            # #
            # # self.place_client.wait_for_server()
            # # self.place_client.send_goal(goal)
            # # self.place_client.wait_for_result()
            # #
            # # print 'finished moving hands away'
            # #
            # drive back and home
            # drive_pose = Pose()
            # drive_pose.position.x = -0.35
            # drive_pose.position.y = 0
            # drive_pose.position.z = 0
            # drive_pose.orientation.x = 0
            # drive_pose.orientation.y = 0
            # drive_pose.orientation.z = 0
            # drive_pose.orientation.w = 1
            # #
            # self.home()
            # self.set_head(-0.3)
            # # rospy.sleep(3.0)
            # # self.home()
            #
            # goal = DriveGoal()
            # goal.drive_goal.header.frame_id = 'base_footprint'
            # goal.drive_goal.header.stamp = rospy.Time.now()
            # goal.drive_goal.pose = drive_pose
            # self.drive_client.wait_for_server()
            # self.drive_client.send_goal(goal)
            # # self.home()
            # # self.set_head(-0.3)
            # # rospy.sleep(1.5)
            # # self.home()
            # self.drive_client.wait_for_result()
            # rospy.sleep(3.0)

            # drive to pose specified in world frame at the end of the action
            # we need insert obstacles again, but without the one we picked up
            pose_msgs = []
            # for i in range(len(req.feature_poses)-1, -1, -1):
            for i in range(len(req.feature_poses)-1, 0, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # add cube obstacle
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y,
                                                req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x,
                                                         req.feature_poses[i].position.y,
                                                         req.feature_poses[i].position.z))

                delta = Point()
                delta = req.feature_poses[i].position - req.feature_poses[0].position

                if math.sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z) < 0.1:
                    continue
                pose_msgs.append(pm.toMsg(frame))
                self.add_cube_obstacle(pm.toMsg(frame))


            # drive up to final location
            drive_pose = Pose()
            drive_pose.position.x = req.parameters[30]
            drive_pose.position.y = req.parameters[31]
            drive_pose.position.z = req.parameters[32]
            drive_pose.orientation.x = req.parameters[33]
            drive_pose.orientation.y = req.parameters[34]
            drive_pose.orientation.z = req.parameters[35]
            drive_pose.orientation.w = req.parameters[36]

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            for i in range(len(pose_msgs)):
                try:
                    self.remove_cube_obstacle(pose_msgs[i])
                except NameError:
                    pass


            print 'finished driving back and homing'

            return 1

        # Place Action
        # feature_poses:
        #   0      - the object frame to apply the pickup action on
        #   1 to n - additional object frames used for obstacle server
        # parameters:
        #   0  - x position for drive goal (object frame)
        #   1  - y position for drive goal (object frame)
        #   2  - z position for drive goal (object frame)
        #   3  - x orientation for drive goal (object frame)
        #   4  - y orientation for drive goal (object frame)
        #   5  - z orientation for drive goal (object frame)
        #   6  - w orientation for drive goal (object frame)
        #   7  - x position for left hand pre-grasp (object frame)
        #   8  - y position for left hand pre-grasp (object frame)
        #   9  - z position for left hand pre-grasp (object frame)
        #   10 - x position for right hand pre-grasp (object frame)
        #   11 - y position for right hand pre-grasp (object frame)
        #   12 - z position for right hand pre-grasp (object frame)
        #   13 - grasp magnitude
        #   14 - x position for hand center (upper trunk frame)
        #   15 - y position for hand center (upper trunk frame)
        #   16 - z position for hand center (upper trunk frame)
        #   17 - x position for left hand retract post put down (base_footprint frame)
        #   18 - y position for left hand retract post put down (base_footprint frame)
        #   19 - z position for left hand retract post put down (base_footprint frame)
        #   20 - x position for right hand retract post put down (base_footprint frame)
        #   21 - y position for right hand retract post put down (base_footprint frame)
        #   22 - z position for right hand retract post put down (base_footprint frame)
        #   23 - 29
        #
        elif req.action_type == 19:
            print 'execute place action'
            print 'params:'
            print req.parameters

            # lift box to make sure you can get close to other boxes (e.g. when stacking)
            center_p_align = Point()
            center_p_align.x = 0.35  # req.parameters[17]
            center_p_align.y = 0.0  # req.parameters[18]
            center_p_align.z = 0.3  # req.parameters[19]

            align_goal = ControlGoal()
            align_goal.reference = 'upper_trunk'
            align_goal.center_goal.position = center_p_align
            align_goal.left_goal.orientation.x = 0.0
            align_goal.left_goal.orientation.y = 1.0
            align_goal.left_goal.orientation.z = 0.0

            align_goal.right_goal.orientation.x = -align_goal.left_goal.orientation.x
            align_goal.right_goal.orientation.y = -align_goal.left_goal.orientation.y
            align_goal.right_goal.orientation.z = -align_goal.left_goal.orientation.z
            align_goal.magnitude = 0.006

            self.force_pos_rotation_client.wait_for_server()
            self.force_pos_rotation_client.send_goal(align_goal)
            self.force_pos_rotation_client.wait_for_result()

            print 'finished aligning box in front of robot for transport'

            pose_msgs = []
            for i in range(len(req.feature_poses)-1, -1, -1):
                feature_frame = pm.fromMsg(req.feature_poses[i])
                print 'feature_frame:'
                print feature_frame

                # add cube obstacle
                rot = PyKDL.Rotation.Quaternion(req.feature_poses[i].orientation.x, req.feature_poses[i].orientation.y,
                                                req.feature_poses[i].orientation.z, req.feature_poses[i].orientation.w)
                unit_x = rot.UnitX()
                unit_y = rot.UnitY()
                if math.fabs(unit_x.x() + unit_x.y()) > math.fabs(unit_y.x() + unit_y.y()):
                    yaw = math.atan2(unit_x.y(), unit_x.x())
                else:
                    yaw = math.atan2(unit_y.y(), unit_y.x())
                orient = PyKDL.Rotation.RotZ(yaw)
                frame = PyKDL.Frame(orient, PyKDL.Vector(req.feature_poses[i].position.x,
                                                         req.feature_poses[i].position.y,
                                                         req.feature_poses[i].position.z))
                pose_msgs.append(pm.toMsg(frame))
                self.add_cube_obstacle(pm.toMsg(frame))

            # drive up to box
            drive_pose_diff = Pose()
            drive_pose_diff.position.x = req.parameters[0]
            drive_pose_diff.position.y = req.parameters[1]
            drive_pose_diff.position.z = req.parameters[2]
            drive_pose_diff.orientation.x = req.parameters[3]
            drive_pose_diff.orientation.y = req.parameters[4]
            drive_pose_diff.orientation.z = req.parameters[5]
            drive_pose_diff.orientation.w = req.parameters[6]

            # check for normalized quaternion:
            mag = math.sqrt(math.pow(drive_pose_diff.orientation.x, 2) + math.pow(drive_pose_diff.orientation.y, 2) +
                            math.pow(drive_pose_diff.orientation.z, 2) + math.pow(drive_pose_diff.orientation.w, 2))
            drive_pose_diff.orientation.x /= mag
            drive_pose_diff.orientation.y /= mag
            drive_pose_diff.orientation.z /= mag
            drive_pose_diff.orientation.w /= mag

            drive_pose_diff_frame = pm.fromMsg(drive_pose_diff)
            temp_frame = feature_frame * drive_pose_diff_frame
            temp_pose = pm.toMsg(temp_frame)

            drive_rot = PyKDL.Rotation.Quaternion(temp_pose.orientation.x, temp_pose.orientation.y,
                                                  temp_pose.orientation.z, temp_pose.orientation.w)
            drive_unit_x = drive_rot.UnitX()
            drive_yaw = math.atan2(drive_unit_x.y(), drive_unit_x.x())
            drive_orient = PyKDL.Rotation.RotZ(drive_yaw)
            drive_frame = PyKDL.Frame(drive_orient, PyKDL.Vector(temp_pose.position.x, temp_pose.position.y,
                                                                 temp_pose.position.z))
            drive_pose = pm.toMsg(drive_frame)

            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'world'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            self.drive_client.wait_for_result()

            for i in range(len(pose_msgs)):
                try:
                    self.remove_cube_obstacle(pose_msgs[i])
                except NameError:
                    pass

            # get placement location in world frame
            placement_goal_pose = Pose()
            placement_goal_pose.position.x = req.parameters[23]
            placement_goal_pose.position.y = req.parameters[24]
            placement_goal_pose.position.z = req.parameters[25]
            placement_goal_pose.orientation.x = req.parameters[26]
            placement_goal_pose.orientation.y = req.parameters[27]
            placement_goal_pose.orientation.z = req.parameters[28]
            placement_goal_pose.orientation.w = req.parameters[29]

            try:
                world_T_upper_trunk = pm.fromTf(self.listener.lookupTransform('/world', '/upper_trunk', rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'transform not available'

            goal_pose = world_T_upper_trunk * placement_goal_pose


            center_p_align = Point()
            center_p_align.x = goal_pose.position.x  # req.parameters[17]
            center_p_align.y = goal_pose.position.y  # req.parameters[18]
            center_p_align.z = goal_pose.position.z + 0.1 + 0.15  # 0.1m as we grasp 10cm above the box center, 0.15m clearance

            align_goal = ControlGoal()
            align_goal.reference = 'upper_trunk'
            align_goal.center_goal.position = center_p_align
            align_goal.left_goal.orientation.x = 0.0
            align_goal.left_goal.orientation.y = 1.0
            align_goal.left_goal.orientation.z = 0.0

            align_goal.right_goal.orientation.x = -align_goal.left_goal.orientation.x
            align_goal.right_goal.orientation.y = -align_goal.left_goal.orientation.y
            align_goal.right_goal.orientation.z = -align_goal.left_goal.orientation.z
            align_goal.magnitude = 0.006

            self.force_pos_rotation_client.wait_for_server()
            self.force_pos_rotation_client.send_goal(align_goal)
            self.force_pos_rotation_client.wait_for_result()

            print 'finished aligning box in front of robot for transport'

            center_p_align = Point()
            center_p_align.x = goal_pose.position.x  # req.parameters[17]
            center_p_align.y = goal_pose.position.y  # req.parameters[18]
            center_p_align.z = goal_pose.position.z + 0.1 # req.parameters[19]

            print 'Placement height: ', center_p_align.z, ' (upper trunk frame)'

            align_goal = ControlGoal()
            align_goal.reference = 'upper_trunk'
            align_goal.center_goal.position = center_p_align
            align_goal.left_goal.orientation.x = 0.0
            align_goal.left_goal.orientation.y = 1.0
            align_goal.left_goal.orientation.z = 0.0

            align_goal.right_goal.orientation.x = -align_goal.left_goal.orientation.x
            align_goal.right_goal.orientation.y = -align_goal.left_goal.orientation.y
            align_goal.right_goal.orientation.z = -align_goal.left_goal.orientation.z
            align_goal.magnitude = 0.006

            self.force_pos_rotation_client.wait_for_server()
            self.force_pos_rotation_client.send_goal(align_goal)
            self.force_pos_rotation_client.wait_for_result()

            print 'finished aligning box in front of robot for transport'

            goal = ControlGoal()
            goal.magnitude = -0.12
            self.squeeze_client.wait_for_server()
            self.squeeze_client.send_goal(goal)
            self.squeeze_client.wait_for_result()

            #drive back and home
            drive_pose = Pose()
            drive_pose.position.x = -0.35
            drive_pose.position.y = 0
            drive_pose.position.z = 0
            drive_pose.orientation.x = 0
            drive_pose.orientation.y = 0
            drive_pose.orientation.z = 0
            drive_pose.orientation.w = 1
            # #
            # self.home()
            # self.set_head(-0.3)
            # # rospy.sleep(3.0)
            # # self.home()
            #
            goal = DriveGoal()
            goal.drive_goal.header.frame_id = 'base_footprint'
            goal.drive_goal.header.stamp = rospy.Time.now()
            goal.drive_goal.pose = drive_pose
            self.drive_client.wait_for_server()
            self.drive_client.send_goal(goal)
            # # self.home()
            # # self.set_head(-0.3)
            # # rospy.sleep(1.5)
            self.home()
            # self.drive_client.wait_for_result()
            # rospy.sleep(3.0)

            print 'finished driving back and homing'

            return 1

        # Brace Action
        # parameters:
        #   0  - arm to use for bracing (0 = 'left', 1 = 'right' or '2 = object')
        #   1  - z-force threshold to detect  #0.2 is a good value
        #   2  - whether or not to preposture before trying to move hand/object towards the ground (0 = false, 1 = true)
        #
        elif req.action_type == 20:
            print 'execute bracing action'
            print 'params:'
            print req.parameters

            bracing_goal = BracingGoal()
            if req.parameters[0] == 0:
                bracing_goal.arm_to_use = 'left'
            elif req.parameters[0] == 1:
                bracing_goal.arm_to_use = 'right'
            elif req.parameters[0] == 2:
                bracing_goal.arm_to_use = 'object'

            bracing_goal.z_force_threshold = req.parameters[1]

            if req.parameters[2] == 0:
                bracing_goal.preposture_first = False
            elif req.parameters[2] == 1:
                bracing_goal.preposture_first = True

            self.bracing_control_client.wait_for_server()
            self.bracing_control_client.send_goal(bracing_goal)
            self.bracing_control_client.wait_for_result()

            print 'finished bracing'

            return 1

        # Restore from Brace Action
        # parameters:
        #   0 - boolean indicating whether or not an object is in hand (0 = false, 1 = true)
        #   no object:
        #       1 - L_TILT value
        #       2 - L_PAN value
        #       3 - L_TWIST value
        #       4 - L_ELBOW value
        #       5 - R_TILT value
        #       6 - R_PAN value
        #       7 - R_TWIST value
        #       8 - R_ELBOW value
        #   with object:
        #       1 - center_goal_x
        #       2 - center_goal_y
        #       3 - center_goal_z
        elif req.action_type == 21:
            print 'execute restoratrion from bracing action'
            print 'params:'
            print req.parameters

            has_object_in_hand = req.parameters[0]

            if has_object_in_hand == 1:
                center_p_align = Point()
                center_p_align.x = req.parameters[1]
                center_p_align.y = req.parameters[2]
                center_p_align.z = req.parameters[3]

                align_goal = ControlGoal()
                align_goal.reference = 'upper_trunk'
                align_goal.center_goal.position = center_p_align
                align_goal.left_goal.orientation.x = 0.0
                align_goal.left_goal.orientation.y = 1.0
                align_goal.left_goal.orientation.z = 0.0

                align_goal.right_goal.orientation.x = -align_goal.left_goal.orientation.x
                align_goal.right_goal.orientation.y = -align_goal.left_goal.orientation.y
                align_goal.right_goal.orientation.z = -align_goal.left_goal.orientation.z
                align_goal.magnitude = 0.006

                self.force_pos_rotation_client.wait_for_server()
                self.force_pos_rotation_client.send_goal(align_goal)
                self.force_pos_rotation_client.wait_for_result()
            else:
                joint_linear_position_request = SetJointLinearPositionsRequest()
                joint_linear_position_request.stamp = rospy.Time.now()
                joint_linear_position_request.numJoints = 12
                joint_linear_position_request.positions = [0] * 12
                joint_linear_position_request.velocities = [0] * 12
                joint_linear_position_request.durations = [3] * 12

                joint_linear_position_request.positions[L_TILT] = req.parameters[1]
                joint_linear_position_request.positions[L_PAN] = req.parameters[2]
                joint_linear_position_request.positions[L_TWIST] = req.parameters[3]
                joint_linear_position_request.positions[L_ELBOW] = req.parameters[4]
                joint_linear_position_request.positions[R_TILT] = req.parameters[5]
                joint_linear_position_request.positions[R_PAN] = req.parameters[6]
                joint_linear_position_request.positions[R_TWIST] = req.parameters[7]
                joint_linear_position_request.positions[R_ELBOW] = req.parameters[8]

                joint_linear_position_request.bitmask = BOTH_ARMS

                response = self.joint_linear_position_client(joint_linear_position_request)
            return 1

        else:
            print 'ACTION TYPE NOT IMPLEMENTED : ', req.action_type


# box is location you want to drive up to (geometry_msgs/Point)
# current pose is robot pose (geometry_msgs/Point)
# nominal_reach_distance is double how close you want to get to box
def drive_up_to_box(self, box, current_pose, nominal_reach_distance):
    # Find drive location at certain radius
    # First you must project robot current pose to correct radius
    # http://stackoverflow.com/questions/1800138/given-a-start-and-end-point-and-a-distance-calculate-a-point-along-a-line
    print 'drive_up_to_box called'
    #
    # nominal_reach_distance = 1.0
    vx = box.x - current_pose.x
    vy = box.y - current_pose.y

    mag = np.sqrt(vx * vx + vy * vy)

    vx /= mag
    vy /= mag

    approach_x = box.x - vx * nominal_reach_distance
    approach_y = box.y - vy * nominal_reach_distance

    print approach_x
    print approach_y

    drive_pose = Pose()
    drive_pose.orientation.x = self.current_pose.pose.orientation.x
    drive_pose.orientation.y = self.current_pose.pose.orientation.y
    drive_pose.orientation.z = self.current_pose.pose.orientation.z
    drive_pose.orientation.w = self.current_pose.pose.orientation.w
    drive_pose.position.x = approach_x
    drive_pose.position.y = approach_y

    goal = DriveGoal()
    goal.drive_goal.header.frame_id = 'world'
    goal.drive_goal.header.stamp = rospy.Time.now()
    goal.drive_goal.pose = drive_pose

    drive_pose_pub = PoseStamped()
    drive_pose_pub.pose = drive_pose
    drive_pose_pub.header.frame_id = 'world'
    drive_pose_pub.header.stamp = rospy.Time.now()

    self.pub.publish(drive_pose_pub)
    self.drive_client.wait_for_server()
    self.drive_client.send_goal(goal)
    self.drive_client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('ubot_action_manager')

    ubot_action_manager = uBotActionManager()

    rospy.spin()

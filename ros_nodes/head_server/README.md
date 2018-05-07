# ubot_control
This package contains the gaze_controller node

Compile normally with catkin_make and run with

`roslaunch head_server server.launch`


## Topics:

/uBot_head/joint_states - Current state of the joints

/uBot_head/joint_desired_states - Desired state of the joints

/uBot_head/command - Incoming command goals for each joint

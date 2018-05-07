# ubot_control
This package contains the gaze_controller node

Compile normally with catkin_make and run with

`roslaunch head_search search.launch`


## Topics:

/head_search/state - Current state of the controller. Refer enum possible_states

/head_search/change_state - Receive flags to change state

## Services

/find_entity - Used to query current model

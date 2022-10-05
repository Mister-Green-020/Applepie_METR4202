# Applepie_METR4202
METR4202 Sem 2 2022 Group Project.

- Understanding ROS Nodes: http://cdn.joshua-wallace.com/metr4202.png
- State machine logic: http://cdn.joshua-wallace.com/statemachine.png

## Nodes
- joint_publisher.py
    - Given a desired pose (3D point to move to), computes the required angles to achieve the pose
    - Subscribes To
        - /new_position (Pose)
    - Publishes To
        - /desired_joint_states (JointState)
- state_machine.py
    - Implements logic to complete move-grab-drop circuit and initialisation of robot
    - Subscribes To
        - /block_positions (TBD)
    - Publishes To
        - /desired_gripper_position (Bool)
        - /new_position (Pose)
        - /ximea_ros/show_rgb (Bool)
- block_planning.py
    - Node to handle behaviour planning including collision checking, colours and position/orientation of blocks
    - Subscribes to
        - /ximea_ros/ximea_XXXXXXXX/image_raw (sensors_msgs/Image)
    - Publishes to
        - /block_positions (TBD)
- gripper.py
    - Node to actuate the gripper when directed by state machine
    - Subscribes To
        - /desired_gripper_position (Bool)
    - Publishes To
        - /gripper_position (Bool) [Redundant topic]


# To do:

- State publisher   (Alex)
    - publish joint angles using python
- State machine     (Jonathen, Josh)
    - basic behaviour planning
- Camera            (Jade, Ben)
    - interpert camera data
        - get coordinates of blocks


## Installing the repo first time

Go into a directory where you can access.

- `git clone https://github.com/Mister-Green-020/Applepie_METR4202.git`
- `cd Applepie_METR4202` or just type `cd App` then tab for autocomplete
- `git remote` to verify that it has installed correctly
- `git switch -c [your name]` for example, `git switch -c alexandra`
- `git add .` - you'll use this to add files
- `git commit -m "Setting up repo"` make an initial commit
- `git push --set-upstream origin [your name]` sets up your remote branch and pushes any changes to the repo



# Terminal Set Up:

```console
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch dynamixel_interface dynamixel_interface_controller.launch

echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```


# Camera Calibration:
```console
catkin_make
rosrun ximea_ros ximea_demo
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_cam/ximea_31702051/image_raw camera:=/ximea_cam/ximea_31702051
```
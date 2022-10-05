# Applepie_METR4202
METR4202 Sem 2 2022 Group Project.



Understanding ROS Nodes: http://cdn.joshua-wallace.com/metr4202.png
State machine logic: http://cdn.joshua-wallace.com/statemachine.png

## Nodes
- joint_angles.py
    - Subscribers:
        - /desired_joint_states
    - Publishers:
        - /joint_states
- state_machine.py (see)
    - Subscribers:
        - /block_positions
    - Publishers:
        - /desired_gripper_position
        - /desired_joint_states
- gripper.py
    - Subscribers:
        - /desired_gripper_position
    - Publishers:
        - /gripper_position
- vision nodes and logic
    - ???


# To do:

- State publisher   (Alex)
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
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

rostopic pub /desired_joint_states [TAB]

echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```


# Camera Calibration:
```console
catkin_make
rosrun ximea_ros ximea_demo
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_cam/ximea_31702051/image_raw camera:=/ximea_cam/ximea_31702051
```

oop
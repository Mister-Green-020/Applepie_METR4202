# Applepie_METR4202
METR4202 Sem 2 2022 Group Project.

- Understanding ROS Nodes: http://cdn.joshua-wallace.com/metr4202.png
- State machine logic: http://cdn.joshua-wallace.com/statemachine.png

## Nodes
- Note XXXXXXXX = 31702051
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


## To do:

- State machine
    - basic behaviour planning
        - flip gripper 90 degrees to detect block colour
- Camera
    - Take picture (metr4202_ximea_ros/.../example_camera.py)
    - interpert camera data
        - get coordinates of blocks
            - frame transform using modern robotics library
        - determine block colour (metr4202_ximea_ros/.../ximea_color_detect.cpp)

## Terminal Set Up:

```console
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch dynamixel_interface dynamixel_interface_controller.launch

rostopic pub /desired_joint_states /msg_JointState [TAB]

echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```


## Camera Calibration:
```console
catkin_make
rosrun ximea_ros ximea_demo
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_cam/ximea_31702051/image_raw camera:=/ximea_cam/ximea_31702051
```

## ArUca Tag Detection
```console
roslaunch ximea_ros ximea_aruco.launch serial:=31702051
```

## GitHub Assistance
### First Installation
Go into a directory where you can access.

- `git clone https://github.com/Mister-Green-020/Applepie_METR4202.git`
- `cd Applepie_METR4202` or just type `cd App` then tab for autocomplete
- `git remote` to verify that it has installed correctly
- `git switch -c [your name]` for example, `git switch -c alexandra`
- `git add .` - you'll use this to add files
- `git commit -m "Setting up repo"` make an initial commit
- `git push --set-upstream origin [your name]` sets up your remote branch and pushes any changes to the repo

### Commiting Files to Remote
For group members to view your files, you need to push them to your branch. Ensure you're in the local repository on your computer, and in the correct branch. Check with `git branch`.

```console
git add .
git commit -m Put a message here saying what you've done
git push
```

You may need to authenticate depending how you've configured your Git. Once done, you should be able to go onto our GitHub repo and see '[Your name] has had recent pushes'.

### Updating Your Local
If you've configured according to the first installation, you will have two branches installed - one called 'main' and another under your name. All changes will be pulled from main. Ensure you're on your own branch, check with `git branch` and you have commited any recent changes.

```console
git switch main
git pull
git switch yourname
git merge origin/main
```
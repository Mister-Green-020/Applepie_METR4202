# Applepie_METR4202
METR4202 Sem 2 2022 Group Project.

- Understanding ROS Nodes: http://cdn.joshua-wallace.com/metr4202.png
- State machine logic: http://cdn.joshua-wallace.com/statemachine.png

## Running With Launch Files
- Start all nodes except state_machine.py
```console
roslaunch applepi master.launch
```
Activate state machines for relevant tasks
- Place blocks in area behind robot arm
```console
roslaunch applepi state.launch
```
- Throw blocks in defined direction
```console
roslaunch applepi throw.launch
```

## Running Without Launch Files
- Setup Commands
```console
sudo pigpiod
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
catkin_make
```
- Provided Packages
```console
roslaunch dynamixel_interface dynamixel_interface_controller.launch
rosrun ximea_ros ximea_demo
roslaunch ximea_ros ximea_aruco.launch serial:=31702051
```
- Our Nodes
```console
python3 ik_node.py
python3 gripper.py
python3 robot_vision.py
python3 state_machine.py
python3 colour_detector.py
```


## Nodes
- joint_publisher.py
    - Given a desired pose (spatial), computes the required angles to achieve the pose
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
- state_machine_throw.py
    - Implements logic to complete move-grab-throw circuit and initialisation of robot
    - Subscribes To
        - /block_positions (TBD)
    - Publishes To
        - /desired_gripper_position (Bool)
        - /new_position (Pose)
        - /ximea_ros/show_rgb (Bool)
- robot_vision.py
    - Vision node to interpret positions of the AruCo tags, validate them as possible and convert to a pose for the robot frame
    - Subscribes to
        - /fiducial_transforms (fiducial_msgs/FiducialTransformArray)
    - Publishes to
        - /block_positions (Pose)
- colour_detection.py
    - Colour detection of the block is done by moving it to a fixed position and taking converting a subset of the large image to determine dominant colour
    - Subscribes to
        - /ximea_ros/ximea_31702051/image_raw (sensor_msgs/Image)
    - Publishes to
        - /block_colour (String)
        - /rgba_colour (ColorRGBA)
- gripper.py
    - Node to actuate the control the gripper
    - Subscribes To
        - /desired_gripper_position (Bool)
    - Publishes To
        - /gripper_position (Bool)


## Robot Set Up:
```console
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch dynamixel_interface dynamixel_interface_controller.launch

rostopic pub /desired_joint_states /msg_JointState [TAB]

echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

## Camera Set Up:
```console
rostopic pub /ximea_ros/show_rgb std_msgs/Bool "data: true" 
```

## Camera Calibration:
```console
catkin_make
rosrun ximea_ros ximea_demo
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_cam/ximea_31702051/image_raw camera:=/ximea_cam/ximea_31702051
```

## ArUco Tag Detection
```console
roslaunch ximea_ros ximea_aruco.launch serial:=31702051
```
## ArUco Tag Locations
```console
rosrun ximea_ros ximea_demo
rostopic echo /fiducial_transforms
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
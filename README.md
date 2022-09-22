# Applepie_METR4202
METR4202 Sem 2 2022 Group Project


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

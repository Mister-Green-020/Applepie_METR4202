# Applepie_METR4202
METR4202 Sem 2 2022 Group Project

## Installing the repo first time

Go into a directory where you can access.

- `git clone https://github.com/Mister-Green-020/Applepie_METR4202.git`
- `cd Applepie_METR4202` or just type `cd App` then tab for autocomplete
- `git remote` to verify that it has installed correctly
- `git switch -c [your name]` for example, `git switch -c alexandra`
- `git add .` - you'll use this to add files
- `git commit -m "Setting up repo"` make an initial commit
- `git push --set-upstream origin [your name]` sets up your remote branch and pushes any changes to the repo



Terminal Set Up:

```console
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch dynamixel_interface dynamixel_interface_controller.launch
'''
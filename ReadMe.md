first launch
```
roslaunch scout_gazebo_sim world.launch
```

second launch
```
export PYTHONPATH="/opt/ros/melodic/lib/python2.7/dist-packages:/home/xt/useful_pkgs/multi_scout/devel/lib/python3/dist-packages"
roslaunch scout_gazebo_sim multi_scout_mini.launch
```

install some prerequisites to use Python3 with ROS.

sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
Prepare catkin workspace

mkdir -p ~/catkin_ws/src; cd ~/catkin_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
Finally compile for Python 3

catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
Do not forget to always source your workspace!

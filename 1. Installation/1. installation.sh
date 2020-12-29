#!/bin/bash


## Create ROS Python workspace.
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules
mkdir -p ~/catkin_ws_python/src && cd ~/catkin_ws_python && catkin_make && source ~/catkin_ws_python/devel/setup.bash && wstool init && wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5 && wstool up && rosdep install --from-paths src --ignore-src -y -r && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so


## Create ROS Packages for the examples.
cd ~/catkin_ws/src && catkin_create_pkg position_setpoints std_msgs roscpp geometry_msgs sensor_msgs mavros_msgs
cd ~/catkin_ws/src && catkin_create_pkg lissajous_px4_pos_ctrl std_msgs roscpp geometry_msgs sensor_msgs mavros_msgs
cd ~/catkin_ws/src && catkin_create_pkg lissajous_att_ctrl std_msgs roscpp geometry_msgs sensor_msgs mavros_msgs
cd ~/catkin_ws_python/src && catkin_create_pkg mocap std_msgs roscpp rospy geometry_msgs sensor_msgs mavros_msgs
cd ~/catkin_ws_python/src && catkin_create_pkg offboard_programs std_msgs roscpp rospy geometry_msgs sensor_msgs mavros_msgs
cd ~/catkin_ws_python/src && catkin_create_pkg tools std_msgs roscpp rospy geometry_msgs sensor_msgs mavros_msgs


## Create some folders for storing the modules.
mkdir ~/catkin_ws_python/src/tools/src/logs
mkdir ~/MAVSDK-PYTHON
mkdir ~/MAVSDK-C++
mkdir ~/Simulation


## Install the Python libraries required by the modules.
pip3 install ipython numpy scipy pandas matplotlib pyyaml


## Install tmux.
sudo apt-get install tmux
cp "~/tiagooliveira/1. Installation/2. tmux.conf" ~/.tmux.conf


## Install MavRouter.
sudo apt install python-future python3-future libtool autoconf
git clone https://github.com/intel/mavlink-router
cd ~/mavlink-router
git submodule update --init --recursive
./autogen.sh && ./configure CFLAGS='-g -O2' --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 --prefix=/usr
make
sudo make install


## Run the bash scripts available in the repository.
cd "~/tiagooliveira/2. Simulation" && source "18. simulation_organizer"
cd "~/tiagooliveira/4. Modules" && source "6. modules_organizer"
cd "~/tiagooliveira/5. Tools" && source "8. tools_organizer"
cd "~/tiagooliveira/7. Examples" && source "7. examples_organizer"


## Install VPRN_CLIENT_ROS.
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-drivers/vrpn_client_ros.git
rosdep install --from-paths .
cd ..
catkin build
source devel/setup.bash


## Complete the installation of MAVSDK Python.
rm -f ~/.local/lib/python3.6/site-packages/mavsdk/system.py
cp "~/tiagooliveira/1. Installation/3. system.py" ~/.local/lib/python3.6/site-packages/mavsdk/system.py
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_001
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_002
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_003
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_004
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_005
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_006
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_007
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_008
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_009
cp ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server ~/.local/lib/python3.6/site-packages/mavsdk/bin/mavsdk_server_010

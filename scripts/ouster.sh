# Steps
# 1. IP Address Configuration
#   - echo "\"192.168.131.20/24\"" > ipaddr
#     curl -X PUT http://os1-xxxxxxxxxxxx/api/v1/system/network/ipv4/override --header "Content-Type: application/json" -T ipaddr
#     rm ipaddr
#   - Note: os1-xxxxxxxxxxxx is the serial number of the Ouster (sometimes os instead of os1)
#
# 2. Install SDK and ROS Driver
#   - 

ws_path="$HOME/catkin_ws"

source /opt/ros/noetic/setup.bash
# Install SDK Dependencies
sudo apt install -y build-essential cmake libeigen3-dev libjsoncpp-dev
sudo apt install -y ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-tf2-geometry-msgs

# Create Workspace
if [ ! -d $ws_path ]
then
    echo -e "\n${on_white}Creating workspace '$ws_path'${color_off}";
    source /opt/ros/$rosdist/setup.bash
    mkdir -p $ws_path/src
    cd $ws_path/src
    catkin_init_workspace
else
    echo -e "\n${on_white}Workspace exists. '$ws_path'${color_off}";
fi

# Clone SDK and ROS Driver
cd $ws_path/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

cd $ws_path
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release

# Noetic (Ubuntu 20.04) Install
# Steps:
#   1. Install CUDA 11.1 - 11.7
#   2. Install Zed SDK >= 3.8
#   3. Clone and build zed-ros-wrapper

# Install ZED SDK
wget https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu20 -O ZED_SDK.run
sudo chmod +x ZED_SDK.run
./ZED_SDK.run

# Clone and Build zed-ros-wrapper
ws_path="$HOME/catkin_ws"
rosdist="noetic"

source /opt/ros/$rosdist/setup.bash
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

cd $ws_path/src
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git

cd $ws_path
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
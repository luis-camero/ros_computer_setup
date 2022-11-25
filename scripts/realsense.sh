cwd=$PWD
on_white='\033[47m'
# Reset
color_off='\033[0m'  
# librealsense Tags:
# - https://github.com/IntelRealSense/librealsense/tags

# realsense-ros Tags: 
# - https://github.com/IntelRealSense/realsense-ros/tags
#
# Steps: 
#   1. Clone: git clone https://github.com/IntelRealSense/librealsense.git
#   2. Install Dependencies:
#       - sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
#       - sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
#   3. Install udev Rules (while in /librealsense):
#       - ./scripts/setup_udev_rules.sh
#   4. Install Kernel Module Patch*:
#       - ./scripts/patch-realsense-ubuntu-lts.sh
#   5. Make SDK:
#       - mkdir build && cd build
#       - cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DFORCE_RSUSB_BACKEND=ON --install
#   6. Install SDK
#       - sudo make uninstall && make clean && make && sudo make install
#
#
# Parse Arguments:
default_ros_tag="2.3.2"
default_sdk_tag="v2.50.0"
default_ws_path="$HOME/catkin_ws"
default_rosdist="noetic"
ros_tag=""
sdk_tag=""
rosdist=""
while getopts "r:s:w:d:" flag
do
    case "${flag}" in
        r) ros_tag="${OPTARG}";;
        s) sdk_tag="${OPTARG}";;
        w) ws_path="${OPTARG}";;
        d) rosdist="${OPTARG}";;
    esac
done

if [ "$ros_tag" != "" ] && [ "$sdk_tag" == "" ]
then
    echo -e "${on_white}Passed in ROS Driver version tag: $ros_tag. \n But, missing SDK version tag:\n\tPass in '-s skd_tag'${color_off}";
    exit 1;
elif [ "$ros_tag" == "" ] && [ "$sdk_tag" != "" ]
then
    echo -e "${on_white}Passed in SDK version tag: $ros_tag. \n But, missing ROS Driver version tag:\n\tPass in '-r ros_tag'${color_off}";
    exit 1;
elif [ "$ros_tag" == "" ] && [ "$sdk_tag" == "" ]
then
    echo -e "${on_white}No parameters passed in. Setting defaults...${color_off}";
    ros_tag="$default_ros_tag";
    sdk_tag="$default_sdk_tag";
fi
echo -e "${on_white}Realsense ROS Driver Version: $ros_tag${color_off}";
echo -e "${on_white}librealsense SDK Version: $sdk_tag${color_off}";

if [ "$ws_path" == "" ]
then
    ws_path="$default_ws_path"
fi
echo -e "${on_white}Workspace: $ws_path${color_off}"

if [ "$rosdist" == "" ]
then
    if [ -z "${ROS_DISTRO}" ]
    then
        rosdist="$default_rosdist"
    else
        rosdist="$ROS_DISTRO"
    fi
fi
echo -e "${on_white}ROS Distro: $rosdist${color_off}"

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

# SDK Install
mkdir -p $ws_path/sdk
cd $ws_path/sdk

## Remove Previous Install
if [ -d $ws_path/sdk/librealsense ]
then
    echo -e "${on_white}Removing previous install of librealsense...${color_off}";
    rm -r $ws_path/sdk/librealsense;
fi
echo -e "${on_white}Downloading librealsense:${color_off}"
git clone https://github.com/IntelRealSense/librealsense.git

## Check if SDK version exists
cd $ws_path/sdk/librealsense
if [ ! $(git tag -l $sdk_tag) ]
then
    echo -e "${on_white}librealsense tag '$sdk_tag' does not exist.${color_off}"
    echo -e "${on_white}Make sure to enter tag exactly as listed here: https://github.com/IntelRealSense/librealsense/tags${color_off}"
    exit 1
else
    echo -e "${on_white}Found '$sdk_tag' in librealsense.${color_off}"
    git checkout tags/$sdk_tag -b $sdk_tag-branch
fi

## Update udev and kernel
echo -e "${on_white}Copy udev rules...${color_off}"
sudo cp $ws_path/sdk/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
echo -e "${on_white}Install kernel module patch...${color_off}"
res=$($ws_path/sdk/librealsense/scripts/patch-realsense-ubuntu-lts.sh <<< "\n\n")

## Build and Install librealsense
echo -e "${on_white}Installing librealsense dependencies...${color_off}"
sudo apt-get update
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
mkdir -p $ws_path/sdk/librealsense/build && cd $ws_path/sdk/librealsense/build
echo -e "${on_white}Building librealsense...${color_off}"
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DFORCE_RSUSB_BACKEND=ON
echo -e "${on_white}Installing librealsense...${color_off}"
sudo make uninstall && make clean && make && sudo make install  prefix="/opt/ros/$rosdist"

cd $cwd

# ROS Driver Install

#*Arch-based distributions
#    Install the base-devel package group.
#    Install the matching linux-headers as well (i.e.: linux-lts-headers for the linux-lts kernel).
#    Navigate to the scripts folder
#    cd ./scripts/
#    Then run the following script to patch the uvc module:
#    ./patch-arch.sh
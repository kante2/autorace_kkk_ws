# < autorace_perception>


# 1. usb_cam 을 사용하기 위해 다음 설치 과정을 진행해야 함.
sudo apt update
sudo apt install -y ros-noetic-usb-cam
# 새 터미널이거나 환경 재적용
source /opt/ros/noetic/setup.bash
cd ~/autorace_kkk_ws
rm -rf build devel
catkin_make

# 2. launch
roslaunch autorace_perception camera_calibration.launch 
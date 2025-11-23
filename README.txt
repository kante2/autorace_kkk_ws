
# inha_kkk ws

ros1 noetic

1.rosbridge 초기설정 
sudo apt update
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
1-2.최종 rosbridge 용
roslaunch rosbridge_server rosbridge_websocket.launch


-----------------------------------------

<정리중,,, >

PERCEPTION/ 부분은, 센서의 전처리, 센서가 넘겨줘야 하는 값들을 넣어놓았다.
CONTROL/ 부분은, 제어에 관한 기능을 넣어놓았다.
DECISION/ 부분은, 미션별로 분기해놓았고, MAIN_NODE가 제어한다.
각 미션별 디버깅을 위해서 LAUNCH로 각각 실행할 수 있게끔 처리하였다.

센서 부분을 키고 -> DECISION에 있는 LAUNCH를 실행하여 각 미션을 확인한다.
값 튜닝은 CONTROL의 YAML을 변경하면 된다.

1117은, old version,,



-----------------------------------------










# ------------install 해야 하는 것들 -------------
sudo apt update
sudo apt install -y ros-noetic-vision-msgs

# 단축키
# source -> ss
echo "alias ss='source devel/setup.bash'" >> ~/.bashrc
source ~/.bashrc

# catkin -> cc
# catkin_make 를 cc로
echo "alias cc='catkin_make'" >> ~/.bashrc
source ~/.bashrc


# slam_gmapping
https://github.com/ros-perception/slam_gmapping.git

# morai_msg
https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git

# tf2
https://github.com/ros/geometry2.git

# f1tenth_gym_ros
git clone -b ros1 https://github.com/f1tenth/f1tenth_gym_ros.git

# docker
불안정 할수 있음, 아직 테스트하기 이전


# morai
계정에 맞는 런처를 이용해야함, 공식사이트에서의 런처로는 로그인이 되지 않음,

cd ~/Downloads/MoraiLauncher_Lin

chmod +x setPermission.sh keylok_install.sh MORAISim.sh MoraiLauncher_Lin.x86_64 LauncherUpdate.sh Install/keylok_install || true

./MoraiLauncher_Lin.x86_64

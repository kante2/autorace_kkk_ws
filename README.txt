
# inha_kkk ws

ros1 noetic

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

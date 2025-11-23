decision부분은 모든 함수를 main이 제어한다.
mission_*_toMain.cpp부분은 노드가 아니라 사실상 미션을 수행하는 함수이다.
-> 즉, 각 미션 파일은 main()을 가지지 않고, mission_*_init(), mission_*_step() 같은 함수만 가지고 있다.
main() 은 딱 한 군데: main_node.cpp




mission_*은 각 미션이 잘 돌아가는지 디버깅 하기 위한 노드이다,.
실행은 mission_*.launch 로 할수 있도록 구성하였다.

mission_*_toMain.cpp → main_node에서 호출하는 모듈용 코드
mission_lane_init(nh, pnh)
mission_lane_step()
main 없음, ros::init 없음

​

mission_*.cpp → 단독 실행하는 개별 노드
자기 안에 int main(...) 있고 ros::init/while loop 가 있음
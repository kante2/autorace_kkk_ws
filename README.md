# INHA_K3 AutoRace ROS Workspace

![ROS](https://img.shields.io/badge/ROS-Noetic-green) ![Python](https://img.shields.io/badge/Python-3.8+-blue)

## 🏆 대회 결과

**AutoRace 2025 스케일카 자율주행 경진대회**
- **팀명**: INHA_K3
- **수상**: 부산시장상 - 최우수상
- **기간**: 2025-09-10 ~ 2025-11-28

---

## 📋 프로젝트 구조

| 디렉토리 | 설명 |
|---------|------|
| **PERCEPTION** | 센서의 전처리, 센서 데이터 처리 |
| **CONTROL** | 제어에 관한 기능 및 YAML 설정 |
| **DECISION** | 미션별 분기, MAIN_NODE에서 제어 |

각 미션별 디버깅을 위해 LAUNCH로 각각 실행할 수 있도록 처리했습니다.

### 사용 흐름
1. 센서 부분을 실행
2. DECISION의 LAUNCH 실행으로 각 미션 확인
3. CONTROL의 YAML 파일을 수정하여 값 튜닝

---

## 🚀 실행 명령어

### Perception Node
```bash
roslaunch perception_node perception.launch
```

### Decision Node
```bash
roslaunch decision_node decision.launch
```

---

## ⚙️ 설치 및 초기 설정

### 필수 패키지 설치

```bash
sudo apt update
sudo apt install -y ros-noetic-vision-msgs
```

### RosBridge 설정

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 단축키 설정

#### Source 단축키 (ss)
```bash
echo "alias ss='source devel/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

#### Catkin 단축키 (cc)
```bash
echo "alias cc='catkin_make'" >> ~/.bashrc
source ~/.bashrc
```

---

## 📦 의존성

### 필수 저장소

| 패키지 | GitHub |
|--------|--------|
| slam_gmapping | https://github.com/ros-perception/slam_gmapping.git |
| MORAI Messages | https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git |
| TF2 | https://github.com/ros/geometry2.git |
| F1tenth Gym | git clone -b ros1 https://github.com/f1tenth/f1tenth_gym_ros.git |

---

## 🐳 Docker

> ⚠️ 현재 불안정 상태 - 테스트 진행 중

---

## 🎮 MORAI 시뮬레이터

### 설정 방법

```bash
cd ~/Downloads/MoraiLauncher_Lin

# 권한 설정
chmod +x setPermission.sh keylok_install.sh MORAISim.sh MoraiLauncher_Lin.x86_64 LauncherUpdate.sh Install/keylok_install || true

# 런처 실행
./MoraiLauncher_Lin.x86_64
```

> 💡 **주의**: 계정에 맞는 런처를 사용해야 합니다. 공식 사이트의 런처로는 로그인이 되지 않을 수 있습니다.


---

**마지막 업데이트**: 2026-01-30

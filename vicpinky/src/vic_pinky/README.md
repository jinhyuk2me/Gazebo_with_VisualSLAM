# Vic Pinky
<img src="/doc/image.png" width="40%" height="30%" title="vicpinky" alt="vicpinky"></img>

Vic Pinky는 교육용 자율주행 로봇 플랫폼입니다. ROS2 Jazzy를 기반으로 하며 Gazebo 시뮬레이션과 실제 로봇 모두에서 동작합니다.

## 주요 기능
- 자율주행 및 내비게이션
- SLAM (동시 위치추정 및 지도작성)
- 레이저 스캔 기반 장애물 회피
- Gazebo 시뮬레이션 지원
- 멀티 로봇 시뮬레이션 지원

---

# PC 설정 (시뮬레이션용)
## 시스템 요구사항
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Gazebo Garden/Harmonic

## 1. Vic Pinky ROS2 패키지 클론
```bash
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
```

## 2. 의존성 패키지 설치
```bash
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 3. 빌드
```bash
cd ~/vicpinky_ws
colcon build --symlink-install
```

## 4. 환경 설정
```bash
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

# 실제 로봇 설정
## 시스템 요구사항
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- RPLidar A1/A2/A3
- 시리얼 포트 권한

## 1. Vic Pinky ROS2 패키지 클론
```bash
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
```

## 2. Gazebo 패키지 제거 (실제 로봇용)
```bash
cd ~/vicpinky_ws/src/vic_pinky
sudo rm -rf vicpinky_gazebo/
```

## 3. 의존성 패키지 설치
```bash
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 4. UDEV 규칙 설정
### 규칙 파일 복사
```bash
cd ~/vicpinky_ws/src/vic_pinky/doc
sudo cp ./99-vic-pinky.rules /etc/udev/rules.d/
```

### UDEV 규칙 적용
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 5. RPLidar 설정
자세한 설정 방법은 다음 문서를 참고하세요:
[RPLidar 설정 가이드](https://github.com/pinklab-art/vic_pinky/blob/main/doc/lidar_setup.md)

## 6. 패키지 빌드
```bash
cd ~/vicpinky_ws
colcon build --symlink-install
```

## 7. 환경 설정
```bash
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


---

# 사용 방법

## 실제 로봇 실행

### 1. 기본 실행
```bash
ros2 launch vicpinky_bringup bringup.launch.xml
```

### 2. 지도 생성 (SLAM)
#### SLAM 툴박스 실행
```bash
ros2 launch vicpinky_navigation map_building.launch.xml
```

#### 지도 시각화 (PC에서만)
```bash
ros2 launch vicpinky_navigation map_view.launch.xml
```

#### 키보드로 로봇 조종
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 지도 저장
```bash
ros2 run nav2_map_server map_saver_cli -f <지도_이름>
```

### 3. 자율주행 (Navigation2)
#### Navigation2 실행
```bash
ros2 launch vicpinky_navigation bringup_launch.xml map:=<지도_이름>
```

#### Navigation2 시각화 (PC에서만)
```bash
ros2 launch vicpinky_navigation nav2_view.launch.xml
```

---

# Gazebo 시뮬레이션

## 시뮬레이션 실행

### 1. 기본 시뮬레이션 실행
```bash
ros2 launch vicpinky_bringup gazebo_bringup.launch.xml
```

### 2. 멀티 로봇 시뮬레이션 (선택사항)
```bash
ros2 launch vicpinky_bringup gazebo_multi_spwan.launch.xml namespace:=robot2 x:=12.0 y:=-16.0
```

## 시뮬레이션에서 지도 생성

### 1. SLAM 툴박스 실행 (시뮬레이션 시간 사용)
```bash
ros2 launch vicpinky_navigation map_building.launch.xml use_sim_time:=true
```

### 2. 지도 시각화 (PC에서만)
```bash
ros2 launch vicpinky_navigation map_view.launch.xml
```

### 3. 키보드로 로봇 조종
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. 지도 저장
```bash
ros2 run nav2_map_server map_saver_cli -f <지도_이름>
```

## 시뮬레이션에서 자율주행

### 1. Navigation2 실행 (시뮬레이션 시간 사용)
```bash
ros2 launch vicpinky_navigation bringup_launch.xml map:=<지도_이름> use_sim_time:=true
```

### 2. Navigation2 시각화 (PC에서만)
```bash
ros2 launch vicpinky_navigation nav2_view.launch.xml
```

---

## 문제 해결

### 자주 발생하는 문제들

#### 1. `laser_filters` 패키지를 찾을 수 없는 경우
```bash
sudo apt update
sudo apt install ros-jazzy-laser-filters -y
```

#### 2. 시리얼 포트 권한 문제
```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

#### 3. Gazebo 실행 시 GPU 관련 오류
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## 패키지 구조

- `vicpinky_bringup`: 로봇 실행 및 시뮬레이션 런치 파일
- `vicpinky_description`: 로봇 URDF/Xacro 모델 정의
- `vicpinky_gazebo`: Gazebo 시뮬레이션 환경 및 월드 파일
- `vicpinky_navigation`: SLAM 및 Navigation2 설정

---

## 라이센스

이 프로젝트는 [라이센스 정보]에 따라 배포됩니다.

## 기여하기

버그 리포트, 기능 요청, 풀 리퀘스트는 언제나 환영합니다!

## 지원

문제가 있거나 질문이 있으시면 [이슈 페이지](https://github.com/pinklab-art/vic_pinky/issues)에 등록해 주세요.

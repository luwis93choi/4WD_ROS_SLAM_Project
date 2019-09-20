# ROS_SLAM_Project

# Target Example : Turtlebot3 SLAM + Gazebo + RViz

# 참조자료 : 

# 2019.09.20.
## 1. Turtlebot 3 설치
### 1.1. 아래 사항에 따라서 기본 준비 실시
#### 1.1.1. 소프트웨어 준비
         • http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/
         • http://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/
         • http://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/

#### 1.1.2. Turtlebot 3 패키지 준비
         • https://github.com/ROBOTIS-GIT/turtlebot3 --- git clone 또는 다운로드를 해서 catkin_ws의 src에 저장함
         • https://github.com/ROBOTIS-GIT/turtlebot3_msgs --- git clone 또는 다운로드를 해서 catkin_ws의 src에 저장함
         • https://github.com/ROBOTIS-GIT/turtlebot3_simulations --- git clone 또는 다운로드를 해서 catkin_ws의 src에 저장함
         • https://github.com/ROBOTIS-GIT/turtlebot3_applications --- git clone 또는 다운로드를 해서 catkin_ws의 src에 저장함
         • https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs --- git clone 또는 다운로드를 해서 catkin_ws의 src에 저장함

### 1.1.3. Turtlebot 3 패키지 빌드
         • catkin_make 
           (* 빌드를 해야지 roslaunch 사용 가능 --- roslaunch를 사용하면 catkin_ws 및 ros 설치 디렉토리를 탐색함)

## 2. Turtlebot 3 실행
### 2.1. 시뮬레이션 대상 결정
         * export TURTLEBOT3_MODEL=burger (burger, waffle_pi 등)
### 2.2. ROSCORE 실행 
         * roscore
### 2.3. Gazebo World 로딩
         * roslaunch turtlebot3_gazebo turtlebot3_world.launch (매우 느리니 기다려야함)
### 2.4. 키보드 조종
         * roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
### 2.5. rosrun map_server map_saver -f ~/map

### 2.5. 각종 실행 에러 대응법
         (1) 없다고 에러 뜨는 모듈에 대해서는 별도 설치 진행
         (2) 없는 모듈 설치법 : sudo apt-get install ros-[버전 명칭]-[모듈 이름]
           * Kinetic 버전 rosserial_python ---> sudo apt-get install ros-kinetic-rosserial-python
           * Kinetic 버전 hls_lfcd_lds_driver (레이저 스캐너 드라이버 모듈) ---> sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
           * Kinetic 버전 gmapping ---> sudo apt-get install ros-kinetic-gmapping
           * Kinetic 버전 map_server ---> sudo apt-get install ros-kinetic-map-server (gmapping은 Kinetic까지만 지원되고 Melodian에서는 Google Cartographer 지원)

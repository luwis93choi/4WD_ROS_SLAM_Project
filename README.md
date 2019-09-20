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

#### 1.1.2. Turtlebot 3 패키지
         • https://github.com/ROBOTIS-GIT/turtlebot3 --- 
         • https://github.com/ROBOTIS-GIT/turtlebot3_msgs
         • https://github.com/ROBOTIS-GIT/turtlebot3_simulations
         • https://github.com/ROBOTIS-GIT/turtlebot3_applications
         • https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs



2. Turtlebot 3 실행
   1) 시뮬레이션 대상 결정 : export TURTLEBOT3_MODEL=burger (burger, waffle_pi 등)
   2) ROSCORE 실행 : roscore
   3) Gazebo World 로딩 : roslaunch turtlebot3_gazebo turtlebot3_world.launch (매우 느리니 기다려야함)
   4) 키보드 조종 : roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   5) rosrun map_server map_saver -f ~/map
   
   * 없다고 에러 뜨는 모듈에 대해서는 별도 설치 진행
   * 없는 모듈 설치법 : sudo apt-get install ros-[버전 명칭]-[모듈 이름]
     (1) Kinetic 버전 rosserial_python ---> sudo apt-get install ros-kinetic-rosserial-python
     (2) Kinetic 버전 hls_lfcd_lds_driver (레이저 스캐너 드라이버 모듈) ---> sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
     (3) Kinetic 버전 gmapping ---> sudo apt-get install ros-kinetic-gmapping
     (4) Kinetic 버전 map_server ---> sudo apt-get install ros-kinetic-map-server (gmapping은 Kinetic까지만 지원되고 Melodian에서는 Google Cartographer 지원)

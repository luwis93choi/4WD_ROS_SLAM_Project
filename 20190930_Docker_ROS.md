# ROS_SLAM_Project - Docker ROS

# 2019.09.30.

### 참고자료
#### https://docs.docker.com/install/linux/docker-ce/ubuntu/
#### https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/docker-installation
#### https://askubuntu.com/questions/1126007/python-cant-find-module-named-apt-pkg
#### https://askubuntu.com/questions/480908/problem-with-update-manager-no-module-named-apt-pkg-in-ubuntu-13-10-having-i
#### https://askubuntu.com/questions/1069087/modulenotfounderror-no-module-named-apt-pkg-error

## 1. Docker Installation / Installation Debugging
### 1.1. Docker Installation
- **sudo apt-get remove docker docker-engine docker.io**
  
  : Old Docker Cleanup / Remove all the older, obsolete, incompatible versions of Docker

- **sudo apt-get update**
  
  : Update apt-get package lists
         
- **sudo apt-get install apt-transport-https ca-certificates curl software-properties-common** 

  : Permit apt-get to access Docker repository through HTTPS
         
- **curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-ke add -**

  : Add Docker's official GPG key
         
- **sudo apt-key fingerprint 0EBF7CD88**

  : Validate the key (If you see the fingerprint below, it is ready to move to the next step)

         pub 4096R/0EBFCD88 2017-02-22

         Key fingerprint = 9DC8 5822 9FC7 DD38 854A E2D8 8D81 803C 0EBF CD88

         uid Docker Release (CE deb)

         sub 4096R/F273FCD8 2017-02-22

- **sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"**

  : Add the repository to the system

### 1.2. Installation Debugging

- **ModuleNotFoundError: No module named 'apt_pkg'** Error Debugging

  This error occurs, mainly because the library "apt_pkg.cpython-35m-x86_64-linux-gnu.so" is not present in /usr/lib/python3/dist-package. Another possible reason is that there are multiple versions of Python installed in the system. In order to handle this error. Following methods are recommended.

   - Copying the necessary library files

      - **cd /usr/lib/python3/dist-packages**

      - **sudo cp apt_pkg.cpython-35m-x86_64-linux-gnu.so apt_pkg.so**


   - Reinstalling related packages

      - **sudo apt-get install --reinstall python3-software-properties-common**

      - **sudo apt-get install --reinstall python-apt pyhton3-apt**


# 2019.10.01.

### 참고자료
#### https://community.oracle.com/thread/4272353
#### http://www.floydhilton.com/docker/2017/03/31/Docker-ContainerHost-vs-ContainerOS-Linux-Windows.html
#### https://stackoverflow.com/questions/32841982/how-can-docker-run-distros-with-different-kernels
#### https://www.freecodecamp.org/news/a-beginner-friendly-introduction-to-containers-vms-and-docker-79a9e3e119b/
#### http://blog.naver.com/PostView.nhn?blogId=hdh7485&logNo=221377371254&parentCategoryNo=&categoryNo=15&viewDate=&isShowPopularPosts=true&from=search
#### https://subicura.com/2017/01/19/docker-guide-for-beginners-1.html
#### https://subicura.com/2016/06/07/zero-downtime-docker-deployment.html
#### https://goodgodgd.github.io/ian-flow/archivers/docker-tutorial
#### http://dev.youngkyu.kr/32
#### https://medium.com/@rookiecj/%EA%B0%91%EC%9E%90%EA%B8%B0-ros-%EA%B7%B8%EB%A6%AC%EA%B3%A0-docker%EA%B0%9C%EB%B0%9C%ED%99%98%EA%B2%BD-5b941c9ff098
#### https://league-cat.tistory.com/354
#### http://crunchtools.com/portability-not-compatibility/
#### https://medium.com/pocs/%EB%A6%AC%EB%88%85%EC%8A%A4-%EC%BB%A4%EB%84%90-%EC%9A%B4%EC%98%81%EC%B2%B4%EC%A0%9C-%EA%B0%95%EC%9D%98%EB%85%B8%ED%8A%B8-1-d36d6c961566
#### https://www.kernel.org/doc/html/v4.17/translations/ko_KR/howto.html

## 2. Docker - Its characteristics
### 2.1. Docker's major characteristics

![Docker - Concept](https://user-images.githubusercontent.com/10843389/66374864-ee396700-e9e6-11e9-95be-8e0105aae615.PNG)

- **Open platform for developing, shipping, and running applications**
  
  You can separate your applications from your infrastructure so you can deliver software quickly

  With Docker, you can manage your infrastructure in the same ways you manage your applications.

  With Docker, you can apply newly patched programs into service while running the current one to keep the service going. (**CI/CD : Continious Integration / Continious Delivery**)

  Docker is based on GO and Linux Kernel.

- **Lightweight & Highly-Portable & Highly-Resourceful Platform**
  
  Docker runs its containers (Image instances) with much less resources and overhead than VM (Virtual Machines). This is because unlike VM, which virtualizes the entire HW architecture of Guest OS and manages resources through hypervisor, Docker isolates its containers as a process and run them directly with host OS kernel. Docker invokes system calls from host OS kernel to run the programs in the containers.

  Docker is highly portable, because they are packaged in standardized forms, containers and images.

  Docker is highly resourceful, because Docker Hub houses many images that users can implement into their system.

- **Docker daemon**

  Docker daemon interacts with its user through Docker API requests and manages Docker objects, such as images, containers, networks, and volumes.

- **Image**
  
  Read-only (Immutable) template with instructions for Docker container

  Image contains OS image setting along with application program's codes, resources, and libraries.

  Image can be managed and built on Dockerfile. It is composed of many layers of instructions and functions.

- **Container**
  
  Instance of Docker Image

  Containers can be changed (Mutable) by its users. Containers are sandoxed environment that can be processed through direct system calls.

  Containers are usually well-isolated from other containers and host machine. Users can re-define and adjust isolation level through "namespace" technology.

### 2.2. Running ROS on Docker and its limitations

- **ROS Docker**
  
  Users can run various versions of ROS on Docker through ROS Docker images.

  Installation methods - http://wiki.ros.org/docker/Tutorials/Docker

- **ROS Docker's limitations**
  
  Docker cannot run GUI-based programs or windows. As a result, any GUI-based events or windows will cause errors when running ROS programs. In order to handle this issue, the user can indirectly access GUI through Xserver (http://wiki.ros.org/docker/Tutorials/GUI)

  Docker is not appropriate when developing high-end single unit of robot. This is because Docker is designed for infrastructure elements, such as servers, rather than client elements. As a result, Docker is more appropriate when developing server elements of the system.

### 2.2. Questions Regarding Docker's compatiblities against HW specifications or Kernel versions

- **Docker is not Magic**

  Although Docker can run various versions of OS and images on different versions of OS. It is not perfect. This is ultimately dependent on HW specifications, especially Kernel versions.

  Since all the Docker containers run on system calls from host OS kernels, if the containers try to call for obsolete or non-existant system calls from host OS, the container will face severe system error.

  Thanks to Docker's highly active open-source community activities, Docker's containers are properly maintained to support numerous versions of OS and kernels. However, if there is a major shift in OS system or kernels, engineers will face to conduct major changes to their containers. Through continuous maintenance, Docker can accomplish Continious Integration / Continious Delivery.



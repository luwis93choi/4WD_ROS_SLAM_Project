# ROS_SLAM_Project - Docker ROS

### 참고자료
#### https://docs.docker.com/install/linux/docker-ce/ubuntu/
#### https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/docker-installation
#### https://askubuntu.com/questions/1126007/python-cant-find-module-named-apt-pkg
#### https://askubuntu.com/questions/480908/problem-with-update-manager-no-module-named-apt-pkg-in-ubuntu-13-10-having-i
#### https://askubuntu.com/questions/1069087/modulenotfounderror-no-module-named-apt-pkg-error

# 2019.09.30.
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

   - Copying apt_pkg.cpython-35m-x86_64-linux-gnu.so
      - cd /usr/lib/python3/dist-packages
      - sudo cp apt_pkg.cpython-35m-x86_64-linux-gnu.so apt_pkg.so

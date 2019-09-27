# ROS_SLAM_Project - USB_CAM PKG Analysis

# Target Example : USB_CAM

### 참고자료
#### http://wiki.ros.org/usb_cam
#### https://github.com/ros-drivers/usb_cam
#### http://wiki.ros.org/image_transport
#### http://wiki.ros.org/libuvc_camera
#### http://wiki.ros.org/uvc_camera
#### https://msadowski.github.io/ros-web-tutorial-pt2-cameras/
#### http://chofukutomi.blogspot.com/2017/01/usb-camera-ros-kinetic-ubuntu-1604.html
#### https://cafe.naver.com/openrt/5963
#### https://answers.ros.org/question/131331/usb_cam-errorwebcam-expected-picture-but-didnt-get-it/
#### https://medium.com/@petehouston/check-usb-camera-supported-output-format-on-linux-1572a28f93c9

# 2019.09.26.
## 1. About ROS USB Camera
  * In order to use USB Camera for ROS systems, we can consider 3 USB Camera packages : 'uvc_camera', 'libuvc_camera', and 'usb_cam'. However, among those 3 packages, 'uvc_camera' is considered depreacated. So, it is recommended to use 'libuvc_camera' or 'usb_camera'.
  * UVC : USB Video Class
  * Installing 'uvc_camera' : sudo apt install ros-kinetic-uvc-camera
  * Installing 'libuvc_camera' : sudo apt install ros-kinetic-libuvc-camera
  * Installing 'usb_cam' : sudo apt install ros-kinetic-usb-cam
  * When it comes to the simplicity, 'usb_cam' is much easier to use and utilize for further uses. 'libuvc_camera' seems to be the latest and most sophisticated ROS Camera package, but it is difficult to set it up and use.

## 2. Running usb-cam package
### 2.1. Installation
         * Installing 'usb_cam' : sudo apt install ros-kinetic-usb-cam
         * Installing image-related packages : sudo apt install ros-kinetic-image-*
         * Installing image view package : sudo apt install ros-kinetic-image-view
### 2.2. Setting up USB camera
         * check device number of USB Camera : ls /dev/video* ---> ex : /dev/video0
         * check the supported video formats of USB Camera : v4l2-ctl --list-formats-ext -d /dev/video[number] ---> check what type of pixel format is supported by the camera
         * Run usb_cam_node : rosrun usb_cam usb_cam_node --- it will not work at the first time, because node setting is not compatible with USB camera's supported settings.
         * Based on the supported formats, change usb_cam_node parameters
         * Change pixel format : rosparam set /usb_cam/pixel_format [format : yuyv, rgb etc]
         * Change video device : rosparam set /usb_cam/video_device /dev/device[number]

## 3. Analyzing usb_cam package
### 3.1. How does it work?
         * 'usb_cam.cpp' defines major functions used for usb_cam_node.
         * First, it accesses camera device driver in order to acquire camera image buffer data.
         * Second, it uses 'image_tranport' package in order to publish/subscribe image as a message. This message is also defined as 'sensor_msgs/Image' type message.
         * Third, by publishing/subscribing Camera image data as a message, other ROS modules or nodes can access this image data through topic names. Here, usb_cam only publishes raw image data. This is because by not specifically defining what type of image it is publishing, other ROS modules or nodes can use raw image to reform it into other forms for its own purposes. It is easier to expand as ROS module if the image is published as raw/not refined.



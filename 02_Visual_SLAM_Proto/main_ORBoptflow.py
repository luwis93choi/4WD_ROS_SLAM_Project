import visual_odometry_ORB_OptFlow_KITTI as vo

VO_KITTI = vo.mono_VO_ORBFlow_KITTI(718.856, 718.856, 718.856, 607.1928, 185.2157,
                                    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/dataset/sequences/00/image_2/',
                                    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/data_odometry_poses/dataset/poses/00.txt')

while True:

    VO_KITTI.img_buffer_feature_tracking(disp_img=True)
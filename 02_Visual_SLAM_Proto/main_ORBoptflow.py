import visual_odometry_ORB_OptFlow_KITTI as vo

from matplotlib import pyplot as plt

VO_KITTI = vo.mono_VO_ORBFlow_KITTI(718.856, 718.856, 718.856, 607.1928, 185.2157,
                                    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/dataset/sequences/00/image_2/',
                                    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/data_odometry_poses/dataset/poses/00.txt')
idx = 0
while True:

    VO_KITTI.img_buffer_feature_tracking(disp_img=True)

    if idx >= 3:
        if VO_KITTI.frame_Skip() == False:

            VO_KITTI.geometric_change_calc()
            VO_KITTI.img_common3Dcloud_triangulate()
            VO_KITTI.pose_estimate()
            VO_KITTI.update()

            # Draw the trajectory 
            plt.title('KITTI Dataset - Monocular Visual Odometry (Relative Translation Scaling)\n[ORB-based Optical Flow]')
            #plt.title('KITTI Dataset - Monocular Visual Odometry (Relative Translation Scaling)\n[ORB-based Optical Flow]\n[g2o Local Bundle Adjustment proto]\n[Levenberg Masquardt Solver / 2 poses with multiple common 3D point clouds]')
            #plt.scatter(pose_T[0][0], pose_T[2][0], c='red')
            plt.plot(VO_KITTI.pose_T[0][0], VO_KITTI.pose_T[2][0], 'ro')

            # Draw the groundtruth
            #plt.scatter(VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][0], VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][2], c='blue')
            plt.plot(VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][0], VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][2], 'bo')

            plt.pause(0.000001)
            plt.show(block=False)

        else:

            print('[FRAME SKIPPED] : Camera is stationary / No need to accumulate pose data')
    else:

        idx += 1

    print('----------------------------------------------------')
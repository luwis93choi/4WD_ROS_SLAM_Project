import visual_odometry_ORB_BF_realsense as vo
import visual_odometry_ORB_BF_KITTI as vo_KITTI

from matplotlib import pyplot as plt

VO_KITTI = vo_KITTI.mono_visual_odom(718.856, 718.856, 718.856, 607.1928, 185.2157,
                                     '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/dataset/sequences/00/image_2/',
                                     '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/KITTI_data_odometry_color/data_odometry_poses/dataset/poses/00.txt')
while True:
    
    pose_T, pprev_prev_cloud, prev_current_cloud = VO_KITTI.calc_trajectory()

    #pose_T = VO_KITTI.calc_trajectory()

    # Draw the trajectory 
    plt.title('KITTI Dataset - Monocular Visual Odometry (Relative Translation Scaling)\n\n[g2o Local Bundle Adjustment proto]\n[Levenberg Masquardt Solver / 2 poses with multiple common 3D point clouds]')
    #plt.scatter(pose_T[0][0], pose_T[2][0], c='red')
    plt.plot(pose_T[0][0], pose_T[2][0], 'ro')

    # Draw the groundtruth
    #plt.scatter(VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][0], VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][2], c='blue')
    plt.plot(VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][0], VO_KITTI.ground_truth_T[VO_KITTI.dataset_current_idx-1][2], 'bo')

    # Draw common cloud points
    for i in range(len(pprev_prev_cloud)):
        plt.plot(pprev_prev_cloud[i][0], pprev_prev_cloud[i][2], color='orange', marker='*')
        plt.plot(prev_current_cloud[i][0], prev_current_cloud[i][2], color='green', marker='*')

    #plt.draw()
    plt.pause(0.000001)
    plt.show(block=False)

VO_KITTI.pose_file.close()
plt.savefig('./KITTI Sequence 00 Test (Graph BA Optimization).png', dpi=300)

import numpy as np
import g2o

class BundleAdjustment(g2o.SparseOptimizer):
    def __init__(self,):

        print('[INFO] Init BA Optimizer')
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        print('[INFO] BA Optimizer Setting : ', solver)
        super().set_algorithm(solver)

    def optimize(self, max_iteration=10):
        super().initialize_optimization()
        super().set_verbose(True)
        super().optimize(max_iteration)

    # Add camera position as the node/vertex of the graph
    def add_pose(self, pose_id, pose_R, pose_T, fx, fy, cx, cy, baseline, fixed=False):
        sbacam = g2o.SBACam(pose_R, pose_T)
        sbacam.set_cam(fx, fy, cx, cy, baseline)

        v_se3 = g2o.VertexCam()
        v_se3.set_id(pose_id)
        v_se3.set_estimate(sbacam)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    # Add 3D point cloud point (3D Landmark) as the node/vertext of the graph
    def add_point(self, point_id, point, fixed=False, marginalized=True):
        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id)
        v_p.set_estimate(point)
        v_p.set_marginalized(marginalized)
        v_p.set_fixed(fixed)
        super().add_vertex(v_p)

    # Add reprojection results as the edge of the graph
    def add_edge(self, point_id, pose_id, 
                 measurement, 
                 information=np.identity(2),
                 robust_kernel=g2o.RobustKernelHuber(np.sqrt(5.991))):  # 95% CI

            edge = g2o.EdgeProjectP2MC()
            edge.set_vertex(0, self.vertex(point_id))
            edge.set_vertex(1, self.vertex(pose_id))
            edge.set_measurement(measurement)
            edge.set_information(information)

            if robust_kernel is not None:
                edge.set_robust_kernel(robust_kernel)

            super().add_edge(edge)

    def get_pose(self, pose_id):
        return self.vertex(pose_id).estimate()

    def get_point(self, point_id):
        return self.vertex(point_id).estimate()
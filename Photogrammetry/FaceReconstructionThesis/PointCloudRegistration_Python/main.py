import open3d as o3d
import numpy as np
import time

treg = o3d.t.pipelines.registration

def draw_registration_result(source, target, transformation):
    source_temp = source.clone()
    target_temp = target.clone()

    source_temp.transform(transformation)

    # This is patched version for tutorial rendering.
    # Use `draw` function for you application.
    o3d.visualization.draw_geometries(
        [source_temp.to_legacy(),
         target_temp.to_legacy()])

demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.t.io.read_point_cloud(demo_icp_pcds.paths[0])
target = o3d.t.io.read_point_cloud(demo_icp_pcds.paths[1])

source = o3d.t.io.read_point_cloud("data/RGBDPoints_test0.ply")
target = o3d.t.io.read_point_cloud("data/RGBDPoints_test1.ply")

# Initial guess transform between the two point-cloud.
# ICP algortihm requires a good initial allignment to converge efficiently.
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])


estimation = treg.TransformationEstimationPointToPlane()

# Search distance for Nearest Neighbour Search [Hybrid-Search is used].
max_correspondence_distance = 0.8

# Initial alignment or source to target transform.
init_source_to_target = trans_init

# Convergence-Criteria for Vanilla ICP
criteria = treg.ICPConvergenceCriteria(relative_fitness=1e+1,
                                       relative_rmse=1e+1,
                                       max_iteration=30)

# Down-sampling voxel-size. If voxel_size < 0, original scale is used.
voxel_size = -1

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
save_loss_log = True

print("Apply Point-to-Plane ICP")
s = time.time()

callback_after_iteration = lambda updated_result_dict : print("Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
    updated_result_dict["iteration_index"].item(),
    updated_result_dict["fitness"].item(),
    updated_result_dict["inlier_rmse"].item()))

reg_point_to_plane = treg.icp(source, target, max_correspondence_distance,
                              init_source_to_target, estimation, criteria, voxel_size,
                              callback_after_iteration)

icp_time = time.time() - s
print("Time taken by Point-To-Plane ICP: ", icp_time)
print("Fitness: ", reg_point_to_plane.fitness)
print("Inlier RMSE: ", reg_point_to_plane.inlier_rmse)

draw_registration_result(source, target, reg_point_to_plane.transformation)
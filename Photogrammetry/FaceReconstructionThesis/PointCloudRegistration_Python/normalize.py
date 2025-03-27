import open3d as o3d
import numpy as np
import time
import os

# for every file in "data", normalize
for file in os.listdir("data"):
    if file.endswith(".ply"):
        pcd = o3d.t.io.read_point_cloud("data/" + file)
        pcd = pcd.to(o3d.core.Device("CUDA:0"))
        pcd.estimate_normals(radius=0.1, max_nn=30)
        # also convert colors to float32
        pcd.point["colors"] = pcd.point["colors"].to(o3d.core.Dtype.Float32) / 255.0
        o3d.t.io.write_point_cloud("data/" + file, pcd)
        print("data/" + file + " normalized !")
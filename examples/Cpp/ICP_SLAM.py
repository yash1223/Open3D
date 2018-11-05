import os
import sys
import numpy as np
import open3d
import time
import math


def visualize_callback(v):
    timera = time.time()
    global i, transform, trajectory, orientation
    try:
        pc = open3d.read_point_cloud(data_dir_path + file_list[i])
    except IndexError:
        return False
    if len(pc_map.points) < 500:
        pc_map.points = pc.points
        return True

    pc = open3d.voxel_down_sample(pc, voxel_size)
    pc.transform(transform)
    open3d.estimate_normals(pc, search_param=search_param)

    info = open3d.registration_icp(
        source=pc,
        target=pc_map,
        max_correspondence_distance=threshold,
        estimation_method=open3d.TransformationEstimationPointToPlane(),
        criteria=open3d.ICPConvergenceCriteria())

    transform = info.transformation.dot(transform)

    trajectory.points = open3d.Vector3dVector(
        np.vstack([np.asarray(trajectory.points),
                   np.array(transform[:3, 3])]))
    pc.transform(info.transformation)
    mesh.transform(info.transformation)
    pc_map.points = open3d.Vector3dVector(
        np.vstack([np.asarray(pc_map.points)[:],
                   np.asarray(pc.points)]))
    pc_map.points = open3d.voxel_down_sample(pc_map, voxel_size).points

    print("Now is the ", i, " Scan.  ", "The size of map is ",
          len(pc_map.points))

    open3d.estimate_normals(pc_map, search_param=search_param)
    i += 1
    print("Time is ", time.time() - timera)
    return True


if __name__ == '__main__':
    data_dir_path = 'point_cloud/'

    file_list = os.listdir(data_dir_path)
    file_list = sorted(file_list)
    print(file_list)
    print(len(file_list))

    pc_map = open3d.read_point_cloud(data_dir_path + file_list[0])
    i = 1

    voxel_size = 0.5
    threshold = 0.5
    search_param = open3d.KDTreeSearchParamHybrid(
        radius=voxel_size * 2, max_nn=30)
    transform = np.eye(4)

    trajectory = open3d.PointCloud()

    mesh = open3d.create_mesh_coordinate_frame(size=5)
    open3d.draw_geometries_with_animation_callback([pc_map, mesh, trajectory],
                                                   visualize_callback,
                                                   width=1000,
                                                   height=600)

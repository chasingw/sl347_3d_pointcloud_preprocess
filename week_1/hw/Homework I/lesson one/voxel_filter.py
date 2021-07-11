# -*- coding: utf-8 -*-
# 实现voxel滤波，并加载数据集中的文件进行验证

import open3d as o3d
from open3d.open3d.geometry import PointCloud
from open3d.open3d.utility import Vector3dVector
import os
import numpy as np
# from pyntcloud import PyntCloud


def load_cloud_from_off(filepath):
    """ load pointcloud from .off format file
        off file format
            OFF
            顶点数 面片数 边数
    Args:
        filepath path of .off pointcloud
    Return:
        pointcloud array
    """

    with open(filepath, 'r') as f:
        off_lines = f.readlines()
    # print(off_lines)
    print(f'points off line 1 = {off_lines[1]}')
    points_num = int(off_lines[1].split(' ')[0])
    points = np.zeros((points_num, 3), dtype=np.float)
    for i in range(points_num):
        # print(off_lines[i+2])
        points[i][0] = float(off_lines[i+2].split(' ')[0])
        points[i][1] = float(off_lines[i+2].split(' ')[1])
        points[i][2] = float(off_lines[i+2].split(' ')[2])
    print(f'points shape = {points.shape}')
    return points

# 功能：对点云进行voxel滤波
# 输入：
#     point_cloud：输入点云
#     leaf_size: voxel尺寸
def voxel_filter(pc_arr, leaf_size = 0.2, type = 'centroid'):
    """ voxel filer for point cloud
    Args:
        pc_arr: input point cloud np array (n, 3)
        leaf_size: voxel size (m)
        type: centroid or random
    Returns:
        filtered_point_cloud: point cloud after filter
    """
    filtered_points = []
    # 作业3
    # 屏蔽开始
    x_min = np.min(pc_arr[:, 0], axis=0)
    y_min = np.min(pc_arr[:, 1], axis=0)
    z_min = np.min(pc_arr[:, 2], axis=0)

    x_max = np.max(pc_arr[:, 0], axis=0)
    y_max = np.max(pc_arr[:, 1], axis=0)
    z_max = np.max(pc_arr[:, 2], axis=0)

    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    
    index_points_dict = {}
    for i in range(point_cloud.shape[0]):
        x = [0]
        y = point_cloud[i][1]
        z = point_cloud[i][2]
        x_index = (x - x_min) // leaf_size
        y_index = (y - y_min) // leaf_size
        z_index = (z - z_min) // leaf_size

        key = x_index * y_range * z_range + y_index * z_range + z_index
        if key not in index_points_dict:
            index_points_dict = []
        index_points_dict[key].append(point_cloud[i])

    filtered_point_cloud = []
    for key, pointlist in index_points_dict.items():
        pointlen = len(pointlist)
        if type == 'centroid':
            pointsum_x = 0
            pointsum_y = 0
            pointsum_z = 0

            for point in pointlist:
                pointsum_x += point[0]
                pointsum_y += point[1]
                pointsum_z += point[2]
            filtered_point_cloud.append(np.array([pointsum_x / pointlen, pointsum_y / pointlen, pointsum_z / pointlen]))
        elif type == 'random':
            random_index = np.random.permutation(pointlen)
            filtered_point_cloud.append(pointlist[random_index])
        else:
            print("error! current not support this methods!")

    # 屏蔽结束

    # 把点云格式改成array，并对外返回
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points

def main():
    # # 从ModelNet数据集文件夹中自动索引路径，加载点云
    # cat_index = 10 # 物体编号，范围是0-39，即对应数据集中40个物体
    # root_dir = '/Users/renqian/cloud_lesson/ModelNet40/ply_data_points' # 数据集路径
    # cat = os.listdir(root_dir)
    # filename = os.path.join(root_dir, cat[cat_index],'train', cat[cat_index]+'_0001.ply') # 默认使用第一个点云
    # point_cloud_pynt = PyntCloud.from_file(file_name)

    # 加载自己的点云文件
    file_name = "/Volumes/maxwell/data/ModelNet/ModelNet40/airplane/train/airplane_0002.off"
    # point_cloud_pynt = PyntCloud.from_file(file_name)
    pc_arr = load_cloud_from_off(file_name)

    # 转成open3d能识别的格式
    # point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    # o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云
    pc_o3d = PointCloud()
    pc_o3d.points = Vector3dVector(pc_arr)
    o3d.visualization.draw_geometries([pc_o3d], window_name="origin") # 显示原始点云

    # 调用voxel滤波函数，实现滤波
    filtered_cloud = voxel_filter(pc_arr, 2.0)
    pc_o3d.points = Vector3dVector(filtered_cloud)
    point_cloud_o3d.points = o3d.utility.Vector3dVector(pc_o3d)
    # 显示滤波后的点云
    o3d.visualization.draw_geometries([pc_o3d], window_name="after filter")

if __name__ == '__main__':
    main()

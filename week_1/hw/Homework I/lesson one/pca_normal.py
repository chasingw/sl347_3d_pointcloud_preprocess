# -*- coding: utf-8 -*-
# 实现PCA分析和法向量计算，并加载数据集中的文件进行验证

import open3d as o3d 
from open3d.open3d.geometry import PointCloud
from open3d.open3d.utility import Vector3dVector

import os
import sys
import numpy as np
import random
# from pyntcloud import PyntCloud

# 功能：计算PCA的函数
# 输入：
#     data：点云，NX3的矩阵
#     correlation：区分np的cov和corrcoef，不输入时默认为False
#     sort: 特征值排序，排序是为了其他功能方便使用，不输入时默认为True
# 输出：
#     eigenvalues：特征值
#     eigenvectors：特征向量
def PCA(data, correlation=False, sort=True):
    """ PCA
    Args:
        data: pointcloud data (array N*3)
        correlation:boolean indicate if use covariance matrix or 
        correlation coefficient matric
        sort: boolean indicate if sort by eigenvalue
    Returns:
        eigenvalues: The eigenvalues computes by SVD
        eigenvectors: The eigenvectors computed by SVD
    """
    # 作业1
    # 屏蔽开始
    
    # 0. Normalize
    N = np.size(data, 0)
    print(f'data = {data}')
    data_mean = np.mean(data, axis=0)
    data_norm = data - data_mean
    print(f'data_norm = {data_norm}')
    if correlation:
        H = np.corrcoef(data_norm, rowvar = False, bias = False)
    else:
        H = np.cov(data_norm, rowvar = False, bias = True)

    # compute SVD
    U, sigma, VT = np.linalg.svd(H)
    eigenvectors = U
    eigenvalues = sigma
    # 屏蔽结束

    if sort:
        sort = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:, sort]

    return eigenvalues, eigenvectors


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


def draw_vector(vector):
    # print(f'vector = {vector}')
    # points = [
    #     [0, 0, 0],
    #     [vector[0], vector[1], vector[2]]
    # ]
    # lines = [[0, 1]]
    # colors = [[1, 0, 0] for i in range(len(lines))]
    # line_set = o3d.geometry.LineSet(
    #     points=o3d.utility.Vector3dVector(points),
    #     lines=o3d.utility.Vector2iVector(lines),
    # )
    # line_set.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([line_set], zoom=0.8)

    points = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
    ]
    lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([line_set], zoom=0.8)

    
def main():
    # 指定点云路径

    if len(sys.argv) == 2:
        object_index = int(sys.argv[1])
    else:
        print("usage:python pca_normal.py [object_number(0~39)]")
        print("use random index")
        object_index = random.randint(0, 40)

    # 物体编号，范围是0-39，即对应数据集中40个物体
    root_dir = '/Volumes/maxwell/data/ModelNet/ModelNet40' # 数据集路径
    obj_names = os.listdir(root_dir)
    obj_name = obj_names[object_index]
    print(f'use object id = {object_index}, name = {obj_name}')
    filepath = os.path.join(root_dir, obj_name,'train', obj_name+'_0001.off') # 默认使用第一个点云

    # 加载原始点云
    # point_cloud_pynt = PyntCloud.from_file("/Users/renqian/Downloads/program/cloud_data/11.ply")
    # pc_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    pc_arr = load_cloud_from_off(filepath)
    pc_o3d = PointCloud()
    pc_o3d.points = Vector3dVector(pc_arr)
    o3d.visualization.draw_geometries([pc_o3d], window_name=obj_name) # 显示原始点云

    # 从点云中获取点，只对点进行处理
    # points = point_cloud_pynt.points
    # print('total points number is:', points.shape[0])

    # 用PCA分析点云主方向
    w, v = PCA(pc_arr)
    point_cloud_vector_0 = v[:, 0] #点云主方向对应的向量
    point_cloud_vector_1 = v[:, 1] 

    draw_vector(point_cloud_vector_0)
    print('the main orientation of this pointcloud is: ', point_cloud_vector)
    print(f'pc_arr shape = {pc_arr.shape}')
    pca_pc_arr = np.dot(pc_arr, point_cloud_vector_0.T)
    print(f'pca_pc_arr shape = {pca_pc_arr.shape}')
    pca_pc_o3d = PointCloud()
    pca_pc_o3d.points = Vector3dVector(pca_pc_arr)
    # TODO: 此处只显示了点云，还没有显示PCA
    o3d.visualization.draw_geometries([pca_pc_o3d])
    
    # 循环计算每个点的法向量
    pcd_tree = o3d.geometry.KDTreeFlann(pc_o3d)
    normals = []
    # 作业2
    # 屏蔽开始

    # 由于最近邻搜索是第二章的内容，所以此处允许直接调用open3d中的函数
    # 0. 遍历点云 最紧邻搜索
    # 1. 计算法向量
    
    # 屏蔽结束
    normals = np.array(normals, dtype=np.float64)
    # TODO: 此处把法向量存放在了normals中
    pc_o3d.normals = o3d.utility.Vector3dVector(normals)
    o3d.visualization.draw_geometries([pc_o3d])


if __name__ == '__main__':
    main()

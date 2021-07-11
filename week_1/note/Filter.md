# Filter



点云滤波

- Noise removal
  - Radius Outlier Removal
  - Statistical Outlier Removal
- Downsampling
- Upsampling/Smoothing/Noise Removal
  - Bilateral Filter
  - MED
  - AVE

[TOC]

## Noise removal

### Radius Outlier Removal

1. For each Point, Find a radius = r neighborhood
2. If number of neighbor k < k_ remove the point

###Statistical Outlier Removal

1. For each point, find a neighborhood
2. Compute its distance to its neighbors d_ij,
3. Model the distances by Gaussian distribution d~N(u, sig)
4. 

## Downsampling

#### Voxel Grid Downsampling

1. Build a voxel grid that contains the point cloud
2. Take one point in each cell



Q1: how to "**take one point**"

Q2: how to "**make it efficient**"



##### How to "take one point" from a cell in the grid?

1. Centroid
   1. For coordinates, compute the average in the cell
   2. For other attributes, voting / average
   3. More accurate but slower
2. Random select
   1. Random select a point in the cell
   2. Less accurate but faster

##### Exact

1. Compute the min or max of the point set {}
2. Determine the voxel grid size 
3. Compute the dimension of the voxel grid
4. Compute voxel index for each point
5. 

![截屏2021-07-04 下午4.04.00](/Users/maxwell/Desktop/截屏2021-07-04 下午4.04.00.png)

**Int32 overflow**

- Example, 3D lidar in autonomous driving. Detection range 200m, voxel grid resolution r=0.05m, assume we crop z to be [-10, 10]
- Dimension of the voxel grid: (20/0.05) * (400/0.05)*(400/0.05)=2.56 * 10_10
- 2_32 = 4.3x10_9 < 2.56x10^10

**Strict Weak Ordering!**

- In cpp, the sort function in <algorithm> supports customized comparator
- However, the comparator should follow the strict weak ordering.

![截屏2021-07-04 下午4.10.26](/Users/maxwell/Desktop/截屏2021-07-04 下午4.10.26.png)

**Voxel Grid Downsampling - Approximated**

Hash table 

#### FPS Farthest Point Sampling

1. Randomly choose a point to be the first FPS point
2. Iterate until we get the desired number of points
   1. for each points in the original point cloud, compute its distance to the nearest FPS point
   2. choose the point with the largest value, add to FPS set

#### Normal Space Sampling (NSS)

Used in Iterative Closest Point (ICP)



#### Learning to Sample

![image-20210704162717868](/Users/maxwell/Library/Application Support/typora-user-images/image-20210704162717868.png)



Chamfer loss

### Upsampling

#### Bilateral Filter - Gaussian Filter

高斯核应用到图片上

![截屏2021-07-04 下午4.37.10](/Users/maxwell/Desktop/截屏2021-07-04 下午4.37.10.png)

双边滤波



##  homework

1. PCA 40个物体

2. 法向量

3. voxel grid random centroid

4. KITTI 

   1. Lidar 上采样
   2. Kitti-depth development kit

   


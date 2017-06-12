# Passing input to `pcl::Feature` class

1. An entire point cloud dataset:
  `setInputCloud(PointCloudConstPtr &)` (_mandatory_)
2. A subset of a point cloud dataset:
  `setIndices(IndicesConstPtr &)` (_optional_)
3. A subset of point neighbours to be used:
  `setSearchSurface (PointCloudConstPtr &)` (_optional_)

Given above functions we have four cases to consider:
1. indices == false && surface == false
  In this case, each feature estimation class will estimate feature at __every__ point of input point cloud dataset.

2. indices == true  && surface == false
  In this case, features will be estimated only at points specified by indices vector from input point cloud dataset.

3. indices == false && surface == true
  In this case features will be estimated at every point of input point cloud dataset, however points from surface point cloud will be used for searching nearest neighbours of input points.

  __Often usecase__: pass downsampled cloud as input cloud, and pass original cloud as a surface set.

4. indices == true  && surface == true
  Features will be estimated only for points from input point cloud specified by indices vector, and surface point cloud will be used for searching nearest neighbours.
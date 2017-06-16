# Normal estimation

Generally having a geometric surface it's usually trivial to infer the direction of the normal at a certain point on the surface as the vector perpendicular to the surface in that point. However, since the point cloud datasets that we acquire represent a set of point samples on the real surface, there are two possibilities:

* obtain the underlying surface from the acquired point cloud dataset, using surface meshing techniques, and then compute the surface normals from the mesh;
* use approximations to infer the surface normals from the point cloud dataset directly.

Here we will address the latter case, that is given a point cloud data set directly compute the surface normals at each point in the cloud. 

## Estimating the normal of a plane tangent to the surface

[reference](http://www.pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation)

The problem of determining the normal to a point on the surface is approximated by the problem of estimating the normal of a plane tangent to the surface, which in turn becomes a least-square plane fitting estimation problem.
The solution for estimating the surface normal is therefore reduced to an analysis of the eigenvectors and eigenvalues (or PCA – Principal Component Analysis) of a covariance matrix created from the nearest neighbors of the query point.

## Choosing the right scale

Normals are estimated from given points surrounding neighbourhood. We have to define the neighbourhood scale with either parameter _k_ (`pcl::Feature::setKSearch`) or _r_ (`pcl::Feature::setRadiusSearch`). There are some things to consider when determining correct values:
  * If we use big _k_ or _r_ values, the neighbour surface may cover adjecent edges which could distort our estimate
  * Too small values in other hand would cause inaccurate estimation

## Speed efficiency

PCL library provides class `pcl::NormalEstimationOMP` which has 100% compatible API with its single-threaded counterpart `pcl::NormalEstimation`. 


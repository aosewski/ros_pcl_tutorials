# ICP (_Iterative Closest Point_)

## Applications
Algorithm used to minimize the difference between two point clouds.

## Overview
One of two input point clouds is kept fixed (the _reference_ point cloud), while the other one (the _source_ point cloud) is transformed to best match the _reference_. The algorithm iteratively revises the transformation (combination of translation and rotation) needed to minimize an error metric, usually the distance from the source to the reference point cloud.

## Algorithm

### Input
* reference and source point clouds,
* initial estimation of the transformation to align the source to the reference (optional),
* criteria for stopping the iterations.

### Output
* refined transformation.

### Algorithm

1. For each point (from the whole set of vertices usually referred to as dense or a selection of pairs of vertices from each model) in the source point cloud, match the closest point in the reference point cloud (or a selected set).
2. Estimate the combination of rotation and translation using a root mean square point to point distance metric minimization technique which will best align each source point to its match found in the previous step after weighting and rejecting outlier points.
3. Transform the source points using the obtained transformation.
4. Iterate (re-associate the points, and so on).


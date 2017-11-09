#include <iostream>
#include <typedefs.h>
#include <pcl/features/normal_3d.h>
#include "preprocessor/SurfaceNormalEstimator.h"

SurfaceNormalsPtr
preprocessor::SurfaceNormalEstimator::run(const PointCloudPtr& input, float radius)
{
    pcl::console::print_info ("Estimating surface normals of point cloud \n");
    pcl::NormalEstimation<PointT, NormalT> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setInputCloud (input);
    SurfaceNormalsPtr normals (new SurfaceNormals);
    normal_estimation.compute (*normals);

    return (normals);
}
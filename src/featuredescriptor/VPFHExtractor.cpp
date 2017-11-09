#include <iostream>
#include "featuredescriptor/VPFHExtractor.h"

GlobalDescriptorsPtr
featuredescriptor::VPFHExtractor::run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals)
{
    pcl::console::print_info ("Extracting features using Viewpoint Feature Histogram \n");
    pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
    vfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    vfh_estimation.setInputCloud (points);
    vfh_estimation.setInputNormals (normals);
    GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
    vfh_estimation.compute (*global_descriptor);

    return (global_descriptor);
}

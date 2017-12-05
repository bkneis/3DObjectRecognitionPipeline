#ifndef STEREORECOGNITION_GRADIENTINTENSITYESTIMATOR_H
#define STEREORECOGNITION_GRADIENTINTENSITYESTIMATOR_H

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <typedefs.h>
#include <pcl/features/intensity_gradient.h>

namespace preprocessor {

    class GradientIntensityEstimator {

    public:
        IntensityGradientPtr
        run(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, SurfaceNormalsPtr normals)
        {
            IntensityGradientPtr cloud_ig (new IntensityGradientCloud);
            pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
            gradient_est.setInputCloud(cloud);
            gradient_est.setInputNormals(normals);
            pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
            gradient_est.setSearchMethod(treept2);
            gradient_est.setRadiusSearch(0.25);
            gradient_est.compute(*cloud_ig);
            return cloud_ig;
        }

    };

}

#endif //STEREORECOGNITION_GRADIENTINTENSITYESTIMATOR_H

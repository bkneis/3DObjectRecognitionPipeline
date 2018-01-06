#ifndef STEREORECOGNITION_SURFACENORMALESTIMATOR_H
#define STEREORECOGNITION_SURFACENORMALESTIMATOR_H


#ifdef USE_OMP
#include <pcl/features/normal_3d_omp.h>
#else
#include <pcl/features/normal_3d.h>
#endif

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <configdefs.h>
#include <pipeline/Config.h>

#include "typedefs.h"

namespace preprocessor {

    template <class PointCloudType>
    class SurfaceNormalEstimator {

    public:

        SurfaceNormalsPtr run(PointCloudType points, Config* conf)
        {
            pcl::console::print_info ("Estimating surface normals of point cloud \n");

#ifdef USE_OMP
            pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
#else
            pcl::NormalEstimation<PointT, NormalT> normal_estimation;
#endif

            normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            double radius = std::stod(conf->get(APPROXIMATIONS, "radius"));
            normal_estimation.setRadiusSearch (radius);
            normal_estimation.setInputCloud (points);
            SurfaceNormalsPtr normals (new SurfaceNormals);
            normal_estimation.compute (*normals);

            return (normals);
        }

    };

}


#endif //STEREORECOGNITION_SURFACENORMALESTIMATOR_H

#ifndef STEREORECOGNITION_SURFACENORMALESTIMATOR_H
#define STEREORECOGNITION_SURFACENORMALESTIMATOR_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <configdefs.h>

namespace preprocessor {

    template <class PointCloudType>
    class SurfaceNormalEstimator {

    public:

        SurfaceNormalsPtr run(PointCloudType points, Config* conf)
        {
            pcl::console::print_info ("Estimating surface normals of point cloud \n");

            pcl::NormalEstimation<PointT, NormalT> normal_estimation;
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

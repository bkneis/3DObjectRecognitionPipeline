#ifndef STEREORECOGNITION_SURFACENORMALESTIMATOR_H
#define STEREORECOGNITION_SURFACENORMALESTIMATOR_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>

namespace preprocessor {

    struct NormalsParameters {
        float radius;
        PointCloudPtr points;
    };

    class SurfaceNormalEstimator {

    public:

        SurfaceNormalsPtr run(void* params)
        {
            NormalsParameters* normalsParams = static_cast<NormalsParameters*>(params);
            pcl::console::print_info ("Estimating surface normals of point cloud \n");
            pcl::NormalEstimation<PointT, NormalT> normal_estimation;
            normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            normal_estimation.setRadiusSearch (normalsParams->radius);
            normal_estimation.setInputCloud (normalsParams->points);
            SurfaceNormalsPtr normals (new SurfaceNormals);
            normal_estimation.compute (*normals);

            return (normals);
        }

    };

}


#endif //STEREORECOGNITION_SURFACENORMALESTIMATOR_H

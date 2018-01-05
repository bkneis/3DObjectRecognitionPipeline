#ifndef STEREORECOGNITION_SURFACENORMALESTIMATOR_H
#define STEREORECOGNITION_SURFACENORMALESTIMATOR_H

#ifdef USE_OMP
#include <pcl/features/normal_3d_omp.h>
#else
#include <pcl/features/normal_3d.h>
#endif

#include <pcl/io/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <chrono>

#include "typedefs.h"

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
            auto t1 = std::chrono::high_resolution_clock::now();

#ifdef USE_OMP
            pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
#else
            pcl::NormalEstimation<PointT, NormalT> normal_estimation;
#endif
            normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            normal_estimation.setRadiusSearch (normalsParams->radius);
            normal_estimation.setInputCloud (normalsParams->points);
            SurfaceNormalsPtr normals (new SurfaceNormals);
            normal_estimation.compute (*normals);

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            std::cout << "normals took " << duration << " micro seconds" << std::endl;

            return (normals);
        }

    };

}


#endif //STEREORECOGNITION_SURFACENORMALESTIMATOR_H

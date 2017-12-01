#ifndef STEREORECOGNITION_FPFHEXTRACTOR_H
#define STEREORECOGNITION_FPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    struct FPFHParameters {
        float featureRadius;
        PointCloudPtr points;
        SurfaceNormalsPtr normals;
        PointCloudPtr keypoints;
    };

    class FPFHExtractor : public FeatureExtractor<LocalDescriptorsPtr> {

    public:

        LocalDescriptorsPtr
        run(void* params)
        {
            pcl::console::print_info ("\nExtracting features using Viewpoint Feature Histogram \n");
            auto fpfhParams = static_cast<FPFHParameters*>(params);
            pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
            fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            fpfh_estimation.setRadiusSearch (fpfhParams->featureRadius);
            fpfh_estimation.setSearchSurface (fpfhParams->points);
            fpfh_estimation.setInputNormals (fpfhParams->normals);
            fpfh_estimation.setInputCloud (fpfhParams->keypoints);
            LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
            fpfh_estimation.compute (*local_descriptors);

            return (local_descriptors);
        }


    };

}

#endif //STEREORECOGNITION_FPFHEXTRACTOR_H

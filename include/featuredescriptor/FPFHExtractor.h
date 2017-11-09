#ifndef STEREORECOGNITION_FPFHEXTRACTOR_H
#define STEREORECOGNITION_FPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    struct FPFHParameters {
        float featureRadius;
    };

    class FPFHExtractor : public FeatureExtractor<LocalDescriptorsPtr> {

    public:

        LocalDescriptorsPtr
        run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals, const PointCloudPtr& keypoints, float feature_radius)
        {
          pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
          fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
          fpfh_estimation.setRadiusSearch (feature_radius);
          fpfh_estimation.setSearchSurface (points);
          fpfh_estimation.setInputNormals (normals);
          fpfh_estimation.setInputCloud (keypoints);
          LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
          fpfh_estimation.compute (*local_descriptors);

          return (local_descriptors);
        }


    };

}

#endif //STEREORECOGNITION_FPFHEXTRACTOR_H

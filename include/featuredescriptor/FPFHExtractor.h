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

    class FPFHExtractor : public FeatureExtractor<LocalDescriptorsPtr, FPFHParameters> {

    public:

        LocalDescriptorsPtr
        run(FPFHParameters params)
        {
          pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
          fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
          fpfh_estimation.setRadiusSearch (params.featureRadius);
          fpfh_estimation.setSearchSurface (params.points);
          fpfh_estimation.setInputNormals (params.normals);
          fpfh_estimation.setInputCloud (params.keypoints);
          LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
          fpfh_estimation.compute (*local_descriptors);

          return (local_descriptors);
        }


    };

}

#endif //STEREORECOGNITION_FPFHEXTRACTOR_H

#ifndef STEREORECOGNITION_FPFHEXTRACTOR_H
#define STEREORECOGNITION_FPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh.h>
#include <configdefs.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    class FPFHExtractor : public FeatureExtractor<LocalDescriptorsPtr> {

    public:

        LocalDescriptorsPtr
        run(PointCloudPtr points, SurfaceNormalsPtr normals, Config* conf)
        {
            auto keypointDetector = new preprocessor::SIFTKeyPointDetector();
            //auto keypoints = keypointDetector->run(points, conf);
            pcl::console::print_info ("\nExtracting features using Viewpoint Feature Histogram \n");
            pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
            fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            double featureRadius = std::stod(conf->get(FPFH, "featureRadius"));
            fpfh_estimation.setRadiusSearch (featureRadius);
            //fpfh_estimation.setSearchSurface (points);
            fpfh_estimation.setInputNormals (normals);
            //fpfh_estimation.setInputCloud (keypoints);
            fpfh_estimation.setInputCloud (points);
            LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
            fpfh_estimation.compute (*local_descriptors);

            return (local_descriptors);
        }


    };

}

#endif //STEREORECOGNITION_FPFHEXTRACTOR_H

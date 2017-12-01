#ifndef STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H
#define STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H

#include <pcl/features/rift.h>
#include <pcl/search/kdtree.h>
#include <preprocessor/GradientIntensityEstimator.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    class RIFTFeatureExtractor : public FeatureExtractor<pcl::PointCloud<pcl::Histogram<32>>> {
    public:
        pcl::PointCloud<pcl::Histogram<32>>
        run(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, SurfaceNormalsPtr normals)
        {
            auto gradientEstimator = new preprocessor::GradientIntensityEstimator();
            IntensityGradientPtr gradients = gradientEstimator->run(cloud, normals);
            pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32>> rift_est;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
            rift_est.setSearchMethod(treept3);
            rift_est.setRadiusSearch(10.0);
            rift_est.setNrDistanceBins(4);
            rift_est.setNrGradientBins(8);
            rift_est.setInputCloud(cloud);
            rift_est.setInputGradient(gradients);
            pcl::PointCloud<pcl::Histogram<32>> rift_output;
            rift_est.compute(rift_output);
            return rift_output;
        }
    };

}

#endif //STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H

//#ifndef STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H
//#define STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H
//
//#include <pcl/features/rift.h>
//#include <pcl/search/kdtree.h>
//#include <preprocessor/GradientIntensityEstimator.h>
//#include "FeatureExtractor.h"
//
//namespace featuredescriptor {
//
//    struct RIFTParameters {
//        PointCloudPtr points;
//        SurfaceNormalsPtr normals;
//    };
//
//    class RIFTFeatureExtractor : public FeatureExtractor<RIFTDescriptor> {
//    public:
//        pcl::PointCloud<pcl::Histogram<32>>
//        run(void* params)
//        {
//            auto riftParams = static_cast<RIFTParameters*>(params);
//            auto gradientEstimator = new preprocessor::GradientIntensityEstimator();
//            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
//            pcl::copyPointCloud(*riftParams->points, *cloud);
//            IntensityGradientPtr gradients = gradientEstimator->run(cloud->points, riftParams->normals);
//            pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFTHistogram> rift_est;
//            pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
//            rift_est.setSearchMethod(treept3);
//            rift_est.setRadiusSearch(10.0);
//            rift_est.setNrDistanceBins(4);
//            rift_est.setNrGradientBins(8);
//            rift_est.setInputCloud(riftParams->points);
//            rift_est.setInputGradient(gradients);
//            RIFTDescriptor rift_output;
//            rift_est.compute(rift_output);
//            return rift_output;
//        }
//    };
//
//}
//
//#endif //STEREORECOGNITION_RIFTFEATUREEXTRACTOR_H

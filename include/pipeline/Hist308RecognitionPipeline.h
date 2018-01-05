#ifndef STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <featuredescriptor/VPFHExtractor.h>
#include <featuredescriptor/CVPFHExtractor.h>
#include <classifier/KNN.h>
#include "RecognitionPipeline.h"

namespace pipeline {

    template<class PointCloudType>
    class Hist308RecognitionPipeline : public RecognitionPipeline<PointCloudType> {

    public:

        explicit Hist308RecognitionPipeline(Config *conf)
                : RecognitionPipeline<PointCloudType>(conf)
        {
            if (!RecognitionPipeline<PointCloudType>::config->getClassificationStartegy().compare(knn)) {
                classifier = new classifier::KNN();
            } else {
                classifier = new classifier::KNN();
            }
            auto models = describeDatabase("../data/random");
            classifier->populateDatabase(models);
        }

        void
        visualize() override {
            pcl::console::print_info("Starting visualizer... Close window to exit\n");
            pcl::visualization::PCLVisualizer vis;
            pcl::visualization::PCLHistogramVisualizer hist_vis;
            vis.addPointCloud(RecognitionPipeline<PointCloudType>::input);

            vis.addPointCloudNormals<PointT, NormalT>(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, 4, 0.02, "normals");

            hist_vis.addFeatureHistogram(*descriptors, 308, "Global descriptor");
            vis.resetCamera();
            vis.spin();
        }

        void
        describe() override {
            // Configure the feature extractor
            if (!RecognitionPipeline<PointCloudType>::config->getFeatureDescriptorStrategy().compare(VPFH)) {
                auto extractor = new featuredescriptor::VPFHExtractor();
                descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
            }
            else if (!RecognitionPipeline<PointCloudType>::config->getFeatureDescriptorStrategy().compare(CVPFH)) {
                auto extractor = new featuredescriptor::CVPFHExtractor();
                descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
            }
            else {
                // Fall back to default
                auto extractor = new featuredescriptor::VPFHExtractor();
                descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
            }
        }

        void
        classify() override {
            auto res = classifier->classify(descriptors);
        }

    protected:

        std::vector<GlobalDescriptorsPtr>
        describeDatabase(std::string clouds)
        {
            std::vector<GlobalDescriptorsPtr> models;
            boost::filesystem::path targetDir(clouds);

            boost::filesystem::directory_iterator it(targetDir), eod;

            BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
                if(boost::filesystem::is_regular_file(p)) {
                    // todo Fix me pass point cloud type not pointer
                    auto cloud = loadPointCloud<PointT>(p.string());
                    // todo hack remove
                    if (cloud->points.size() > 10001) {
                        // not random cloud
                        RecognitionPipeline<PointCloudType>::input = preprocessor::removeOutliers(cloud, 0.3, 300);
                    }
                    else {
                        RecognitionPipeline<PointCloudType>::input = cloud;
                    }
                    RecognitionPipeline<PointCloudType>::estimateSurfaceNormals();
                    describe();
                    models.push_back(descriptors);
                }
            }

            return models;
        }

        GlobalDescriptorsPtr descriptors;
        classifier::Classifier<GlobalDescriptorsPtr>* classifier;

    };
}

#endif //STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
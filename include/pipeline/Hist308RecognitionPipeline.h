#ifndef STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <featuredescriptor/VPFHExtractor.h>
#include <featuredescriptor/CVPFHExtractor.h>
#include "RecognitionPipeline.h"

namespace pipeline {

    template<class PointCloudType>
    class Hist308RecognitionPipeline : public RecognitionPipeline<PointCloudType> {

    public:

        explicit Hist308RecognitionPipeline(Config *conf)
                : RecognitionPipeline<PointCloudType>(conf) {
            // Initialise classifier and populate database
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
            } else if (!RecognitionPipeline<PointCloudType>::config->getFeatureDescriptorStrategy().compare(CVPFH)) {
                auto extractor = new featuredescriptor::CVPFHExtractor();
                descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
            } else {
                // Fall back to default
                auto extractor = new featuredescriptor::VPFHExtractor();
                descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
            }
        }

        void
        classify() override {

        }

    protected:

        GlobalDescriptorsPtr descriptors;

    };
}

#endif //STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
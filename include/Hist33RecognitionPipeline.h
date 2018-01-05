#ifndef STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H

#include <featuredescriptor/FPFHExtractor.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "RecognitionPipeline.h"

namespace pipeline {

    template<class PointCloudType>
    class Hist33RecognitionPipeline: public RecognitionPipeline<PointCloudType> {

    public:

        explicit Hist33RecognitionPipeline(Config* conf)
                : RecognitionPipeline<PointCloudType>(conf)
        {

        }

        void
        visualize() override
        {
            pcl::console::print_info ("Starting visualizer... Close window to exit\n");
            pcl::visualization::PCLVisualizer vis;
            pcl::visualization::PCLHistogramVisualizer hist_vis;
            vis.addPointCloud (RecognitionPipeline<PointCloudType>::input);

            vis.addPointCloudNormals<PointT,NormalT> (RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, 4, 0.02, "normals");

            hist_vis.addFeatureHistogram (*descriptors, 33, "Local descriptor");
            vis.resetCamera ();
            vis.spin();
        }

        void
        describe() override
        {
            // Add keypoints here
            auto extractor = new featuredescriptor::FPFHExtractor();
            descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
        }

        void
        classify() override
        {

        }

    protected:
        LocalDescriptorsPtr descriptors;

    };

}


#endif //STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H

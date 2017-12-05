#ifndef STEREORECOGNITION_RECOGNITIONPIPELINE_H
#define STEREORECOGNITION_RECOGNITIONPIPELINE_H

#include <vector>
#include <preprocessor/KeypointDetector.h>
#include <featuredescriptor/FeatureExtractor.h>
#include <preprocessor/SurfaceNormalEstimator.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <featuredescriptor/VPFHExtractor.h>
#include "typedefs.h"
#include "Config.h"

using namespace preprocessor;
using namespace featuredescriptor;


template <class FeatureDescriptorsPtr>
class RecognitionPipeline {

public:

    RecognitionPipeline(Config* config) {
        this->config = config;
    }

    void
    run(PointCloudPtr input)
    {
        void* voidNormalParams;
        if (!config->getNormalsStrategy().compare("approximations")) {
            auto normalsParams = new NormalsParameters();
            normalsParams->radius = 0.2;
            normalsParams->points = input;
            voidNormalParams = static_cast<void*>(normalsParams);
        }
        auto normals = this->normalEstimator->run(voidNormalParams);

        void* voidKeypointParams;
        if (!config->getKeypointStrategy().compare("sift")) {
            auto siftParams = new SiftParameters();
            siftParams->minScale = 0.01f;
            siftParams->numOctaves = 3;
            siftParams->numScalesPerOctave = 4;
            siftParams->minContrast = 0.001f;
            siftParams->points = input;
            voidKeypointParams = static_cast<void*>(siftParams);
        }
        auto keypoints = this->keypointDetector->run(voidKeypointParams);

        void* voidFeatureDescriptorParams;
        if (!config->getFeatureDescriptorStrategy().compare("VPFH")) {
            auto vpfhParams = new VPFHParameters();
            vpfhParams->points = input;
            vpfhParams->normals = normals;
            voidFeatureDescriptorParams = static_cast<void*>(vpfhParams);
        }
        auto descriptors = this->featureExtractor->run(voidFeatureDescriptorParams);

        pcl::console::print_info ("Starting visualizer... Close window to exit\n");
        pcl::visualization::PCLVisualizer vis;
        pcl::visualization::PCLHistogramVisualizer hist_vis;
        vis.addPointCloud (input);

        vis.addPointCloudNormals<PointT,NormalT> (input, normals, 4, 0.02, "normals");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
        // vis.addPointCloud (keypoints, red, "keypoints");
        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

        hist_vis.addFeatureHistogram (*descriptors, 308, "Global descriptor");
        vis.resetCamera ();
        vis.spin();
    }

    void
    setKeypointDetector(KeypointDetector<PointCloudPtr>* keypointDetector)
    {
        this->keypointDetector = keypointDetector;
    }

    void
    setSurfaceNormalEstimator(SurfaceNormalEstimator* normalEstimator)
    {
        this->normalEstimator = normalEstimator;
    }

    void
    setFeatureExtractor(FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor)
    {
        this->featureExtractor = featureExtractor;
    }

private:

    KeypointDetector<PointCloudPtr>* keypointDetector;
    SurfaceNormalEstimator* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;
    Config* config;
};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

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
#include <featuredescriptor/CVPFHExtractor.h>
#include <featuredescriptor/FPFHExtractor.h>
#include "typedefs.h"
#include "Config.h"
#include "loader.h"

using namespace preprocessor;
using namespace featuredescriptor;


template <class FeatureDescriptorsPtr, class PointCloudType>
class RecognitionPipeline {

public:
    RecognitionPipeline(Config* config) {
        this->config = config;
    }

    FeatureDescriptorsPtr
    extract(PointCloudType input)
    {
        // Configure the normal estimation strategy
        void* voidNormalParams;
        if (!config->getNormalsStrategy().compare("approximations")) {
            auto normalsParams = new NormalsParameters();
            normalsParams->radius = 0.2;
            normalsParams->points = input;
            voidNormalParams = static_cast<void*>(normalsParams);
            setSurfaceNormalEstimator(new SurfaceNormalEstimator());
        }
        auto normals = this->normalEstimator->run(voidNormalParams);

        // Configure the keypoint detection strategy
        // TODO don't calculate keypoints unless needed by feature descritor
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

        // Configure the feature descriptor
        void* voidFeatureDescriptorParams;
        if (!config->getFeatureDescriptorStrategy().compare("VPFH")) {
            auto vpfhParams = new VPFHParameters();
            vpfhParams->points = input;
            vpfhParams->normals = normals;
            voidFeatureDescriptorParams = static_cast<void*>(vpfhParams);
        }
        else if (!config->getFeatureDescriptorStrategy().compare("CVPFH")) {
            auto cvpfhParams = new CVPFHParameters();
            cvpfhParams->points = input;
            cvpfhParams->normals = normals;
            voidFeatureDescriptorParams = static_cast<void*>(cvpfhParams);
        }
        else if (!config->getFeatureDescriptorStrategy().compare("FPFH")) {
            auto fpfhParams = new FPFHParameters();
            fpfhParams->featureRadius = 0.2;
            fpfhParams->points = input;
            fpfhParams->normals = normals;
            fpfhParams->keypoints = keypoints;
            voidFeatureDescriptorParams = static_cast<void*>(fpfhParams);
        }
        auto descriptors = this->featureExtractor->run(voidFeatureDescriptorParams);

        return descriptors;
    }

    void
    run(PointCloudType input)
    {
        auto descriptor = extract(input);
        std::vector<FeatureDescriptorsPtr> database;
        //database.push_back(extract(loadPointCloud(std::string("/home/arthur/cloud"), std::string(".pcd"))));
        //database.push_back(extract(loadPointCloud(std::string("/home/arthur/cloud8"), std::string(".pcd"))));

//        pcl::console::print_info ("Starting visualizer... Close window to exit\n");
//        pcl::visualization::PCLVisualizer vis;
//        pcl::visualization::PCLHistogramVisualizer hist_vis;
//        vis.addPointCloud (input);
//
//        vis.addPointCloudNormals<PointT,NormalT> (input, normals, 4, 0.02, "normals");
//
//        pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
//        vis.addPointCloud (keypoints, red, "keypoints");
//        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
//
//        // hist_vis.addFeatureHistogram (*descriptors, 308, "Global descriptor");
//        vis.resetCamera ();
//        vis.spin();
    }

    void
    setKeypointDetector(KeypointDetector<PointCloudType>* keypointDetector)
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
    KeypointDetector<PointCloudType>* keypointDetector;
    SurfaceNormalEstimator* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;
    Config* config;
};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

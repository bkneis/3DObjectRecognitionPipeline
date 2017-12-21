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
#include <classifier/Classifier.h>
#include <classifier/KNN.h>
#include <chrono>
#include <preprocessor/filters.h>
#include "typedefs.h"
#include "Config.h"
#include "configdefs.h"
#include "utils.h"

using namespace preprocessor;
using namespace featuredescriptor;
using namespace classifier;

#define DEBUG_MODE false

template <class FeatureDescriptorsPtr, class PointCloudType>
class RecognitionPipeline {

public:
    RecognitionPipeline(Config* config) {
        this->config = config;
        setClassifier(new KNN());
        loadModels();
    }

    FeatureDescriptorsPtr
    extract(PointCloudType input)
    {
        // Configure the normal estimation strategy
        if (!config->getNormalsStrategy().compare(APPROXIMATIONS)) {
            setSurfaceNormalEstimator(new SurfaceNormalEstimator<PointCloudType>());
        }
        else {
            // Fall back to default
            setSurfaceNormalEstimator(new SurfaceNormalEstimator<PointCloudType>());
        }

        // Compute the surface normals
        auto normals = this->normalEstimator->run(input, config);

//        // Configure the feature extractor
//        if (!config->getFeatureDescriptorStrategy().compare(VPFH)) {
//            setFeatureExtractor(new VPFHExtractor());
//        }
//        else if (!config->getFeatureDescriptorStrategy().compare(CVPFH)) {
//            setFeatureExtractor(new CVPFHExtractor());
//        }
////        else if (!config->getFeatureDescriptorStrategy().compare(FPFH)) {
////            setFeatureExtractor(new FPFHExtractor());
////        }
//        else {
//            // Fall back to default
//            setFeatureExtractor(new VPFHExtractor());
//        }

        setFeatureExtractor(new FPFHExtractor());

        // Compute the feature descriptor
        auto descriptors = this->featureExtractor->run(input, normals, config);

        // Show the point cloud if in debug mode
        if (DEBUG_MODE) {
            visualize(input, normals, descriptors);
        }

        return descriptors;
    }

    void
    visualize(PointCloudType input, SurfaceNormalsPtr normals, FeatureDescriptorsPtr descriptors)
    {
        pcl::console::print_info ("Starting visualizer... Close window to exit\n");
        pcl::visualization::PCLVisualizer vis;
        pcl::visualization::PCLHistogramVisualizer hist_vis;
        vis.addPointCloud (input);

        vis.addPointCloudNormals<PointT,NormalT> (input, normals, 4, 0.02, "normals");

        hist_vis.addFeatureHistogram (*descriptors, 308, "Global descriptor");
        vis.resetCamera ();
        vis.spin();
    }

    void
    run(PointCloudType input)
    {
        // Perform segmentation and remove background
        input = preprocessor::removeOutliers(input, 0.3, 300);

        auto t1 = std::chrono::high_resolution_clock::now();

        FeatureDescriptorsPtr descriptor = extract(input);
        // todo move this to loadModels
        models.push_back(descriptor);
        this->classifier->populateDatabase(models);
        auto test = this->classifier->classify(descriptor);

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        cout << "Pipeline took " << duration << " micro seconds" << endl;
    }

    void
    setSurfaceNormalEstimator(SurfaceNormalEstimator<PointCloudType>* normalEstimator)
    {
        this->normalEstimator = normalEstimator;
    }

    void
    setFeatureExtractor(FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor)
    {
        this->featureExtractor = featureExtractor;
    }

    void
    setClassifier(Classifier<FeatureDescriptorsPtr>* classifier)
    {
        this->classifier = classifier;
    }

private:

    void
    loadModels()
    {
        int size = 20;
        std::vector<PointCloudType> clouds = generateRandomClouds(size, 100, 100);
        for (int i = 0; i < size; i++) {
            auto desc = extract(clouds.at(i));
            models.push_back(desc);
        }
        PointCloudPtr cloud (new PointCloud);
        pcl::io::loadPCDFile ("/home/arthur/memo/cloud3.pcd", *cloud);
        models.push_back(extract(cloud));
    }

    std::vector<FeatureDescriptorsPtr> models;
    SurfaceNormalEstimator<PointCloudType>* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;
    Classifier<FeatureDescriptorsPtr>* classifier;
    Config* config;
};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

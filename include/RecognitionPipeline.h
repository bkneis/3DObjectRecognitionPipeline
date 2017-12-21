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

template <class PointCloudType>
class RecognitionPipeline {

public:
    RecognitionPipeline(Config* config) {
        this->config = config;
        if (isLocal()) {
            localClassifier = new KNN();
            localClassifier->populateDatabase("../data/");
        }
//        else {
//            globalClassifier = new KNN();
//            globalClassifier->populateDatabase("../data/");
//        }
    }

    bool
    isLocal()
    {
        return (!config->getFeatureDescriptorStrategy().compare(FPFH));
    }

    void
    extract(PointCloudType input)
    {
        SurfaceNormalsPtr normals;

        // Configure the normal estimation strategy
        if (!config->getNormalsStrategy().compare(APPROXIMATIONS)) {
            auto estimator = new SurfaceNormalEstimator<PointCloudType>();
            normals = estimator->run(input, config);
        }
        else {
            // Fall back to default
            auto estimator = new SurfaceNormalEstimator<PointCloudType>();
            normals = estimator->run(input, config);
        }

        // Configure the feature extractor
        if (!config->getFeatureDescriptorStrategy().compare(VPFH)) {
            auto extractor = new VPFHExtractor();
            auto descriptor = extractor->run(input, normals, config);

            // Show the point cloud if in debug mode
            if (DEBUG_MODE) {
                visualize(input, normals, descriptor);
            }
            globalClassifier->classify(descriptor);
        }
        else if (!config->getFeatureDescriptorStrategy().compare(CVPFH)) {
            auto extractor = new CVPFHExtractor();
            auto descriptor = extractor->run(input, normals, config);
            // Show the point cloud if in debug mode
            if (DEBUG_MODE) {
                visualize(input, normals, descriptor);
            }
            globalClassifier->classify(descriptor);
        }
        else if (!config->getFeatureDescriptorStrategy().compare(FPFH)) {
            auto extractor = new FPFHExtractor();
            auto descriptor = extractor->run(input, normals, config);
            // Show the point cloud if in debug mode
            if (DEBUG_MODE) {
                visualize(input, normals, descriptor);
            }
            localClassifier->classify(descriptor);
        }
        else {
            // Fall back to default
            auto extractor = new VPFHExtractor();
            auto descriptor = extractor->run(input, normals, config);
            // Show the point cloud if in debug mode
            if (DEBUG_MODE) {
                visualize(input, normals, descriptor);
            }
            globalClassifier->classify(descriptor);
        }

    }

    void
    visualize(PointCloudType input, SurfaceNormalsPtr normals, GlobalDescriptorsPtr descriptors)
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
    visualize(PointCloudType input, SurfaceNormalsPtr normals, LocalDescriptorsPtr descriptors)
    {
        pcl::console::print_info ("Starting visualizer... Close window to exit\n");
        pcl::visualization::PCLVisualizer vis;
        pcl::visualization::PCLHistogramVisualizer hist_vis;
        vis.addPointCloud (input);

        vis.addPointCloudNormals<PointT,NormalT> (input, normals, 4, 0.02, "normals");

        hist_vis.addFeatureHistogram (*descriptors, 33, "Global descriptor");
        vis.resetCamera ();
        vis.spin();
    }

    void
    run(PointCloudType input)
    {
        // Perform segmentation and remove background
        input = preprocessor::removeOutliers(input, 0.3, 300);

        auto t1 = std::chrono::high_resolution_clock::now();

        extract(input);

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        cout << "Pipeline took " << duration << " micro seconds" << endl;
    }

private:

    Classifier<LocalDescriptorsPtr>* localClassifier;
    Classifier<GlobalDescriptorsPtr>* globalClassifier;
    Config* config;
};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

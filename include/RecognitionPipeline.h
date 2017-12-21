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
#include <featuredescriptor/RIFTFeatureExtractor.h>
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
        PointCloudPtr keypoints = NULL;
        // Configure the normal estimation strategy
        void* voidNormalParams;
        if (!config->getNormalsStrategy().compare(APPROXIMATIONS)) {
            auto normalsParams = new NormalsParameters();
            normalsParams->radius = std::stof(config->get(APPROXIMATIONS, "radius"));
            normalsParams->points = input;
            voidNormalParams = static_cast<void*>(normalsParams);
            setSurfaceNormalEstimator(new SurfaceNormalEstimator());
        }
        auto normals = this->normalEstimator->run(voidNormalParams);

        // Configure the feature descriptor
        void* voidFeatureDescriptorParams;
        if (!config->getFeatureDescriptorStrategy().compare(VPFH)) {
            auto vpfhParams = new VPFHParameters();
            vpfhParams->points = input;
            vpfhParams->normals = normals;
            voidFeatureDescriptorParams = static_cast<void*>(vpfhParams);
            setFeatureExtractor(new VPFHExtractor());
        }
        else if (!config->getFeatureDescriptorStrategy().compare(CVPFH)) {
            auto cvpfhParams = new CVPFHParameters();
            cvpfhParams->points = input;
            cvpfhParams->normals = normals;
            voidFeatureDescriptorParams = static_cast<void*>(cvpfhParams);
        }
        else if (!config->getFeatureDescriptorStrategy().compare(FPFH)) {
            void* voidKeypointParams;
            // Configure the keypoint detection strategy
            if (!config->getKeypointStrategy().compare(SIFT)) {
                auto siftParams = new SiftParameters();
                siftParams->minScale = std::stof(config->get(SIFT, "minScale"));
                siftParams->numOctaves = stoi(config->get(SIFT, "numOctaves"));
                siftParams->numScalesPerOctave = stoi(config->get(SIFT, "numScalesPerOctave"));
                siftParams->minContrast = std::stof(config->get(SIFT, "minContrast"));
                siftParams->points = input;
                voidKeypointParams = static_cast<void*>(siftParams);
                setKeypointDetector(new SIFTKeyPointDetector());
            }
            keypoints = this->keypointDetector->run(voidKeypointParams);
            auto fpfhParams = new FPFHParameters();
            fpfhParams->featureRadius = std::stof(config->get(FPFH, "featureRadius"));
            fpfhParams->points = input;
            fpfhParams->normals = normals;
            fpfhParams->keypoints = keypoints;
            voidFeatureDescriptorParams = static_cast<void*>(fpfhParams);
        }
//        else if (!config->getFeatureDescriptorStrategy().compare("RIFT")) {
//            auto riftParams = new RIFTParameters();
//            riftParams->points = input;
//            riftParams->normals = normals;
//            voidFeatureDescriptorParams = static_cast<void*>(riftParams);
//        }
        GlobalDescriptorsPtr descriptors = this->featureExtractor->run(voidFeatureDescriptorParams);

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
    }

    std::vector<FeatureDescriptorsPtr> models;
    KeypointDetector<PointCloudType>* keypointDetector;
    SurfaceNormalEstimator* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;
    Classifier<FeatureDescriptorsPtr>* classifier;
    Config* config;
};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

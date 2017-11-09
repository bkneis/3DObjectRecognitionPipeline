#ifndef STEREORECOGNITION_RECOGNITIONPIPELINE_H
#define STEREORECOGNITION_RECOGNITIONPIPELINE_H

#include <vector>
#include <preprocessor/KeypointDetector.h>
#include <featuredescriptor/FeatureExtractor.h>
#include <preprocessor/SurfaceNormalEstimator.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include "typedefs.h"

using namespace preprocessor;
using namespace featuredescriptor;


template <class PointCloudType, class FeatureDescriptorsPtr, typename NormalsParameters, typename KeypointParameters, typename FeatureParameters>
class RecognitionPipeline {

public:

    void
    run(PointCloudPtr input)
    {
      this->normalParams.points = input;
      auto normals = this->normalEstimator->run(normalParams);

      this->keypointParams.points = input;
      auto keypoints = this->keypointDetector->run(this->keypointParams);

      this->featureParams.points = input;
      this->featureParams.normals = normals;
      auto descriptors = this->featureExtractor->run(this->featureParams);

      pcl::console::print_info ("Starting visualizer... Close window to exit\n");
      pcl::visualization::PCLVisualizer vis;
      pcl::visualization::PCLHistogramVisualizer hist_vis;
      vis.addPointCloud (input);

      vis.addPointCloudNormals<PointT,NormalT> (input, normals, 4, 0.02, "normals");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
      vis.addPointCloud (keypoints, red, "keypoints");
      vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

      // hist_vis.addFeatureHistogram (*descriptors, 308, "Global descriptor");
      vis.resetCamera ();
      vis.spin ();
    }

    void
    setKeypointDetector(KeypointDetector<PointCloudType, KeypointParameters>* keypointDetector, KeypointParameters params)
    {
      this->keypointDetector = keypointDetector;
      this->keypointParams = params;
    }

    void
    setSurfaceNormalEstimator(SurfaceNormalEstimator* normalEstimator, NormalsParameters params)
    {
      this->normalEstimator = normalEstimator;
      this->normalParams = params;
    }

    void
    setFeatureExtractor(FeatureExtractor<FeatureDescriptorsPtr, FeatureParameters>* featureExtractor, FeatureParameters params)
    {
      this->featureExtractor = featureExtractor;
      this->featureParams = params;
    }

private:

    KeypointDetector<PointCloudType, KeypointParameters>* keypointDetector;
    KeypointParameters keypointParams;

    SurfaceNormalEstimator* normalEstimator;
    NormalsParameters normalParams;

    FeatureExtractor<FeatureDescriptorsPtr, FeatureParameters>* featureExtractor;
    FeatureParameters featureParams;

};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

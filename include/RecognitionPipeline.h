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


template <class PointCloudType, typename KeypointParams, class FeatureDescriptorsPtr>
class RecognitionPipeline {

public:

    void
    run(PointCloudPtr input)
    {
      auto normals = this->normalEstimator->run(input, 0.2);
      SiftParameters siftParams = { 0.01f, 3, 4, 0.001f };
      auto keypoints = this->keypointDetector->run(input, normals, siftParams);
      auto descriptors = this->featureExtractor->run(input, normals);

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
    setKeypointDetector(KeypointDetector<PointCloudType, KeypointParams>* keypointDetector)
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

    KeypointDetector<PointCloudType, KeypointParams>* keypointDetector;
    SurfaceNormalEstimator* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;

};

#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

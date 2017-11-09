#ifndef STEREORECOGNITION_RECOGNITIONPIPELINE_H
#define STEREORECOGNITION_RECOGNITIONPIPELINE_H

#include <vector>
#include <preprocessor/KeypointDetector.h>
#include <featuredescriptor/FeatureExtractor.h>
#include <preprocessor/SurfaceNormalEstimator.h>
#include "typedefs.h"

using namespace preprocessor;
using namespace featuredescriptor;

template <class PointCloudType, typename KeypointParams, class FeatureDescriptorsPtr>
class RecognitionPipeline {

public:
    void run(PointCloudPtr cloud);
    void setKeypointDetector(KeypointDetector<PointCloudType, KeypointParams>* keypointDetector);
    void setSurfaceNormalEstimator(SurfaceNormalEstimator* normalEstimator);
    void setFeatureExtractor(FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor);

private:
    KeypointDetector<PointCloudType, KeypointParams>* keypointDetector;
    SurfaceNormalEstimator* normalEstimator;
    FeatureExtractor<FeatureDescriptorsPtr>* featureExtractor;

};


#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

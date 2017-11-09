#include <typedefs.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include "RecognitionPipeline.h"

using namespace preprocessor;
using namespace featuredescriptor;

void
RecognitionPipeline::run(PointCloudPtr input)
{
  auto normals = this->normalEstimator->run(input, 0.2);
  SiftParameters siftParams = { 0.01f, 3, 4, 0.001f };
  auto keypoints = this->keypointDetector->run(input, normals, siftParams);
  auto descriptors = this->featureExtractor->run(keypoints, normals);
}

void
RecognitionPipeline::setKeypointDetector(KeypointDetector* keypointDetector)
{
  this->keypointDetector = keypointDetector;
}

void
RecognitionPipeline::setSurfaceNormalEstimator(SurfaceNormalEstimator* normalEstimator)
{
  this->normalEstimator = normalEstimator;
}

void
RecognitionPipeline::setFeatureExtractor(FeatureExtractor* featureExtractor)
{
  this->featureExtractor = featureExtractor;
}

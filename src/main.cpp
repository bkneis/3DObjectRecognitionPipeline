#include <RecognitionPipeline.h>
#include <preprocessor/SurfaceNormalEstimator.h>
#include "featuredescriptor/VPFHExtractor.h"

using namespace preprocessor;
using namespace featuredescriptor;

int main() {

  // Create the vision processing pipeline
  auto pipeline = new RecognitionPipeline();

  // Determine which processing elements to add based on recognition algorithm
  pipeline->add(new SurfaceNormalEstimator());
  pipeline->add(new SIFTKeyPointDetector());
  pipeline->add(new VPFHExtractor());

  // Run the pipeline
  pipeline->run();

}
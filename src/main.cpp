#include <RecognitionPipeline.h>
#include <SurfaceNormalEstimator.h>

int main() {

  // Create the vision processing pipeline
  auto pipeline = new RecognitionPipeline();

  // Determine which processing elements to add based on recognition algorithm
  pipeline->add(new KeyPointDetector());
  pipeline->add(new SurfaceNormalEstimator());

  // Run the pipeline
  pipeline->run();

}
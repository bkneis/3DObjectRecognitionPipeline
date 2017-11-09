#include <RecognitionPipeline.h>
#include <pcl/io/pcd_io.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include "featuredescriptor/VPFHExtractor.h"

using namespace preprocessor;
using namespace featuredescriptor;

int
main(int arc, char** argv)
{

  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded input point cloud %s (%lu points)\n", argv[1], cloud->size ());

  // Create the vision processing pipeline
  auto pipeline = new RecognitionPipeline<PointCloudPtr, SiftParameters, GlobalDescriptorsPtr>();

  // Determine which processing elements to add based on recognition algorithm
  pipeline->setSurfaceNormalEstimator(new SurfaceNormalEstimator());
  pipeline->setKeypointDetector(new SIFTKeyPointDetector());
  pipeline->setFeatureExtractor(new VPFHExtractor());

  // Run the pipeline
  pipeline->run(cloud);

  return 0;
}
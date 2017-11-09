#include <RecognitionPipeline.h>
#include <pcl/io/pcd_io.h>
#include <preprocessor/SIFTKeyPointDetector.h>
#include "featuredescriptor/VPFHExtractor.h"
#include <yaml-cpp/yaml.h>
#include <ConfigReader.h>

using namespace preprocessor;
using namespace featuredescriptor;

int
main(int arc, char** argv)
{
    if (arc < 3) {
        pcl::console::print_info ("Not enough arguments, Example usage: ./%s pipeline.yaml cloud.pcd \n", argv[1]);
        return 0;
    }

    auto config = ConfigReader::get(argv[1]);

    // Load the input file
    PointCloudPtr cloud (new PointCloud);
    pcl::io::loadPCDFile (argv[2], *cloud);
    pcl::console::print_info ("Loaded input point cloud %s (%lu points)\n", argv[1], cloud->size());

    // Create the vision processing pipeline
    auto pipeline = new RecognitionPipeline<PointCloudPtr, GlobalDescriptorsPtr, NormalsParameters, SiftParameters, VPFHParameters>();

    NormalsParameters normalParams = { 0.2 };
    normalParams.radius = 0.2;

    SiftParameters siftParams = { 0.01f, 3, 4, 0.001f };

    VPFHParameters vpfhParams; // = {};

    // Determine which processing elements to add based on recognition algorithm
    pipeline->setSurfaceNormalEstimator(new SurfaceNormalEstimator(), normalParams);
    pipeline->setKeypointDetector(new SIFTKeyPointDetector(), siftParams);
    pipeline->setFeatureExtractor(new VPFHExtractor(), vpfhParams);

    // Run the pipeline
    pipeline->run(cloud);

  return 0;
}
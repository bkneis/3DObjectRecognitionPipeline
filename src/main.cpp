#include <pcl/io/pcd_io.h>
#include <acquisition/StereoVision.h>
#include <typedefs.h>
#include <pipeline/Config.h>
#include "pipeline/RecognitionPipelineFactory.h"

using namespace acquisition;

int
main(int arc, char** argv)
{
    if (arc < 3) {
        pcl::console::print_info ("Not enough arguments, Example usage: ./%s pipeline.yaml cloud.pcd \n", argv[0]);
        return 0;
    }

//    auto sensor = new StereoVision;
//    sensor->run();

    auto config = Config::create(argv[1]);

    // Load the input file
    PointCloudPtr cloud (new PointCloud);
    pcl::io::loadPCDFile (argv[2], *cloud);
    pcl::console::print_info ("Loaded input point cloud %s (%lu points)\n", argv[2], cloud->size());

    // Create the vision processing pipeline
    auto pipeline = pipeline::RecognitionPipelineFactory<PointCloudPtr>::create(config);

    // Run the pipeline
    pipeline->run(cloud);

  return 0;
}
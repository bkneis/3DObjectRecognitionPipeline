#include <RecognitionPipeline.h>
#include <pcl/io/pcd_io.h>
#include <ConfigReader.h>
#include <preprocessor/filters.h>
#include <acquisition/StereoVision.h>

using namespace acquisition;
using namespace preprocessor;
using namespace featuredescriptor;

int
main(int arc, char** argv)
{
    if (arc < 3) {
        pcl::console::print_info ("Not enough arguments, Example usage: ./%s pipeline.yaml cloud.pcd \n", argv[0]);
        return 0;
    }

    auto sensor = new StereoVision;

    sensor->run();

    auto config = ConfigReader::get(argv[1]);

    // Load the input file
    PointCloudPtr cloud (new PointCloud);
    pcl::io::loadPCDFile (argv[2], *cloud);
    pcl::console::print_info ("Loaded input point cloud %s (%lu points)\n", argv[2], cloud->size());

    // Create the vision processing pipeline
    auto pipeline = new RecognitionPipeline<GlobalDescriptorsPtr, PointCloudPtr>(config);

    pipeline->setKeypointDetector(new SIFTKeyPointDetector());
    pipeline->setFeatureExtractor(new VPFHExtractor());
    pipeline->setClassifier(new KNN());

    cloud = preprocessor::removeOutliers(cloud, 0.3, 300);

    // Run the pipeline
    pipeline->run(cloud);

  return 0;
}
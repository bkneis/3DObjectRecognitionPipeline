#include <RecognitionPipeline.h>
#include <pcl/io/pcd_io.h>
#include <ConfigReader.h>
#include <preprocessor/filters.h>

using namespace preprocessor;
using namespace featuredescriptor;

int
main(int arc, char** argv)
{
    if (arc < 3) {
        pcl::console::print_info ("Not enough arguments, Example usage: ./%s pipeline.yaml cloud.pcd \n", argv[0]);
        return 0;
    }

    auto config = ConfigReader::get(argv[1]);

    // Load the input file
    PointCloudPtr cloud (new PointCloud);
    pcl::io::loadPCDFile (argv[2], *cloud);
    pcl::console::print_info ("Loaded input point cloud %s (%lu points)\n", argv[2], cloud->size());

    // Create the vision processing pipeline
    auto pipeline = new RecognitionPipeline<LocalDescriptorsPtr, PointCloudPtr>(config);

    pipeline->setSurfaceNormalEstimator(new SurfaceNormalEstimator());
    pipeline->setKeypointDetector(new SIFTKeyPointDetector());
    pipeline->setFeatureExtractor(new FPFHExtractor());
    //pipeline->setFeatureExtractor(new CVPFHExtractor());
    //pipeline->setFeatureExtractor(new VPFHExtractor());

//    std::cerr << "Cloud before filtering: " << std::endl;
//    for (size_t i = 0; i < cloud->points.size (); ++i)
//        std::cerr << "    " << cloud->points[i].x << " "
//                  << cloud->points[i].y << " "
//                  << cloud->points[i].z << std::endl;

    // cloud = preprocessor::thresholdDepth(cloud, -10, -20);
    cloud = preprocessor::removeOutliers(cloud, 0.3, 300);

//    std::cerr << "Cloud after filtering: " << std::endl;
//    for (size_t i = 0; i < cloud->points.size (); ++i)
//        std::cerr << "    " << cloud->points[i].x << " "
//                  << cloud->points[i].y << " "
//                  << cloud->points[i].z << std::endl;

    // Run the pipeline
    pipeline->run(cloud);

  return 0;
}
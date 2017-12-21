

#include <pcl/io/pcd_io.h>
#include <sstream>
#include <Config.h>
#include <typedefs.h>
#include <RecognitionPipeline.h>

template <typename PointT = pcl::PointXYZRGB>
std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>>
generateRandomClouds (int num, uint32_t width, uint32_t height)
{
    std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> clouds;

    for (int i = 0; i < num; i++)
    {
        boost::shared_ptr<pcl::PointCloud<PointT>> cloud (new pcl::PointCloud<PointT>);
        // Generate pointcloud data
        cloud->width = width;
        cloud->height = height;
        cloud->points.resize (cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
        }

        clouds.push_back(cloud);
    }

    return clouds;
}

int
main(int argc, char** argv)
{
    auto config = Config::create(argv[1]);

    // Create the vision processing pipeline
    auto pipeline = new RecognitionPipeline<PointCloudPtr>(config);

    auto randomClouds = generateRandomClouds(20, 100, 100);
    for (size_t i = 0; i < randomClouds.size(); i++) {
        std::ostringstream ss;
        ss << "../data/random/cloud";
        ss << i;
        ss << ".pcd";
        // TODO convert the cloud to descriptor
        pcl::io::savePCDFileASCII (ss.str(), *randomClouds.at(i));
    }
}

#ifndef IO_H_
#define IO_H_

#include "typedefs.h"

#include <pcl/io/pcd_io.h>

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
loadPointCloud(std::string filename)
{
   boost::shared_ptr<pcl::PointCloud<PointT> > output (new pcl::PointCloud<PointT>);
   pcl::io::loadPCDFile (filename, *output);
   pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
   return (output);
}

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

#endif

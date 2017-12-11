#ifndef IO_H_
#define IO_H_

#include "typedefs.h"

#include <pcl/io/pcd_io.h>

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
loadPointCloud(std::string filename, std::string suffix)
{
   boost::shared_ptr<pcl::PointCloud<PointT> > output (new pcl::PointCloud<PointT>);
   filename.append (suffix);
   pcl::io::loadPCDFile (filename, *output);
   pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
   return (output);
}

#endif

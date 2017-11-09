#include <preprocessor/SIFTKeyPointDetector.h>

PointCloudPtr
preprocessor::SIFTKeyPointDetector::run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals,
                                        SiftParameters params)
{
  pcl::console::print_info ("Detecting keypoints using 3D SIFT with surface normals \n");
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  sift_detect.setScales (params.minScale, params.numOctaves, params.numScalesPerOctave);
  sift_detect.setMinimumContrast (params.minContrast);
  sift_detect.setInputCloud (points);
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloud);
  pcl::copyPointCloud (keypoints_temp, *keypoints);

  return (keypoints);
}
#ifndef STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H
#define STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H

#include <typedefs.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <configdefs.h>
#include "KeypointDetector.h"

namespace preprocessor {

    class SIFTKeyPointDetector : public KeypointDetector<PointCloudPtr> {

    public:

        PointCloudPtr
        run(PointCloudPtr points, Config* conf)
        {
            pcl::console::print_info ("Detecting keypoints using 3D SIFT with surface normals \n");

            // Get the parameters for SIFT from the pipeline config
            float minScale = std::stof(conf->get(SIFT, "minScale"));
            int numOctaves = std::stoi(conf->get(SIFT, "numberOfOctaves"));
            int numScalesPerOctave = std::stoi(conf->get(SIFT, "numberOfScalesPerOctave"));
            float minContrast = std::stof(conf->get(SIFT, "minContrast"));

            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
            sift_detect.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            sift_detect.setScales (minScale, numOctaves, numScalesPerOctave);
            sift_detect.setMinimumContrast (minContrast);
            sift_detect.setInputCloud (points);
            pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
            sift_detect.compute (keypoints_temp);
            PointCloudPtr keypoints (new PointCloud);
            pcl::copyPointCloud (keypoints_temp, *keypoints);

            return (keypoints);
        }

    };

}

#endif //STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H

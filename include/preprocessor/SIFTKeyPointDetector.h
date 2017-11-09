#ifndef STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H
#define STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H

#include <typedefs.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "KeypointDetector.h"

namespace preprocessor {

    struct SiftParameters {
        float minScale;
        int numOctaves;
        int numScalesPerOctave;
        float minContrast;
        PointCloudPtr points;
    };

    class SIFTKeyPointDetector : public KeypointDetector<PointCloudPtr, SiftParameters> {

    public:

        PointCloudPtr
        run(SiftParameters params)
        {
            pcl::console::print_info ("Detecting keypoints using 3D SIFT with surface normals \n");
            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
            sift_detect.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            sift_detect.setScales (params.minScale, params.numOctaves, params.numScalesPerOctave);
            sift_detect.setMinimumContrast (params.minContrast);
            sift_detect.setInputCloud (params.points);
            pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
            sift_detect.compute (keypoints_temp);
            PointCloudPtr keypoints (new PointCloud);
            pcl::copyPointCloud (keypoints_temp, *keypoints);

            return (keypoints);
        }

    };

}

#endif //STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H

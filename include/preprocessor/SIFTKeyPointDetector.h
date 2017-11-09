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
    };

    class SIFTKeyPointDetector : public KeypointDetector<PointCloudPtr, SiftParameters> {

    public:

        PointCloudPtr run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals, SiftParameters params);

    };

}


#endif //STEREORECOGNITION_SIFTKEYPOINTDETECTOR_H

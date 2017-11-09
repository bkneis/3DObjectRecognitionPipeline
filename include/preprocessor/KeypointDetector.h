#ifndef STEREORECOGNITION_KEYPOINTDETECTOR_H
#define STEREORECOGNITION_KEYPOINTDETECTOR_H

#include <typedefs.h>

namespace preprocessor {

    template <class PointCloudPtr, typename params>
    class KeypointDetector {

    public:

        virtual PointCloudPtr run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals, params args) = 0;

    };

}

#endif //STEREORECOGNITION_KEYPOINTDETECTOR_H

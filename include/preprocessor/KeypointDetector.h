#ifndef STEREORECOGNITION_KEYPOINTDETECTOR_H
#define STEREORECOGNITION_KEYPOINTDETECTOR_H

#include <typedefs.h>
#include <pipeline/Config.h>

namespace preprocessor {

    template <class PointCloudType>
    class KeypointDetector {

    public:
        virtual PointCloudType run(PointCloudType points, Config* conf) = 0;

    };

}

#endif //STEREORECOGNITION_KEYPOINTDETECTOR_H

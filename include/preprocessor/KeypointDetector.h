#ifndef STEREORECOGNITION_KEYPOINTDETECTOR_H
#define STEREORECOGNITION_KEYPOINTDETECTOR_H

#include <typedefs.h>

namespace preprocessor {

    template <class PointCloudPtr>
    class KeypointDetector {

    public:

        virtual PointCloudPtr run(void* params) = 0;

    };

}

#endif //STEREORECOGNITION_KEYPOINTDETECTOR_H

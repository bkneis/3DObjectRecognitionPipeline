#ifndef STEREORECOGNITION_KEYPOINTDETECTOR_H
#define STEREORECOGNITION_KEYPOINTDETECTOR_H

#include <typedefs.h>

namespace preprocessor {

    template <class PointCloudPtr, typename Parameters>
    class KeypointDetector {

    public:

        virtual PointCloudPtr run(Parameters params) = 0;

    };

}

#endif //STEREORECOGNITION_KEYPOINTDETECTOR_H

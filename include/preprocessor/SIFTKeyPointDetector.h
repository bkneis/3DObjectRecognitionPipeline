#ifndef STEREORECOGNITION_KEYPOINTDETECTOR_H
#define STEREORECOGNITION_KEYPOINTDETECTOR_H

#include "Node.h"

namespace preprocessor {

    class SIFTKeyPointDetector : public Node {
    public:
        void* run(void* args);
    };

}



#endif //STEREORECOGNITION_KEYPOINTDETECTOR_H

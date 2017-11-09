#ifndef STEREORECOGNITION_FEATUREEXTRACTOR_H
#define STEREORECOGNITION_FEATUREEXTRACTOR_H

#include <typedefs.h>

namespace featuredescriptor {

    template <class DescriptorsPtr>
    class FeatureExtractor {

    public:

        virtual DescriptorsPtr run(void* params) = 0;

    };

}


#endif //STEREORECOGNITION_FEATUREEXTRACTOR_H

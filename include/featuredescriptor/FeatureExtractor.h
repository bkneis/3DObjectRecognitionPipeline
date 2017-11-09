#ifndef STEREORECOGNITION_FEATUREEXTRACTOR_H
#define STEREORECOGNITION_FEATUREEXTRACTOR_H

#include <typedefs.h>

namespace featuredescriptor {

    template <class DescriptorsPtr, typename Parameters>
    class FeatureExtractor {

    public:

        virtual DescriptorsPtr run(const Parameters) = 0;

    };

}


#endif //STEREORECOGNITION_FEATUREEXTRACTOR_H

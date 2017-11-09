#ifndef STEREORECOGNITION_FEATUREEXTRACTOR_H
#define STEREORECOGNITION_FEATUREEXTRACTOR_H

#include <typedefs.h>

namespace featuredescriptor {

    template <class DescriptorsPtr>
    class FeatureExtractor {

    public:

        virtual DescriptorsPtr run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals) = 0;

    };

}


#endif //STEREORECOGNITION_FEATUREEXTRACTOR_H

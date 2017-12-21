#ifndef STEREORECOGNITION_FEATUREEXTRACTOR_H
#define STEREORECOGNITION_FEATUREEXTRACTOR_H

#include <typedefs.h>
#include <Config.h>

namespace featuredescriptor {

    template <class DescriptorsPtr>
    class FeatureExtractor {

    public:
        virtual DescriptorsPtr run(PointCloudPtr points, SurfaceNormalsPtr normals, Config* conf) = 0;

    };

}


#endif //STEREORECOGNITION_FEATUREEXTRACTOR_H

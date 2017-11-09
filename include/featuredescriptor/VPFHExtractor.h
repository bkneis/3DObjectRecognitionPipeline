#ifndef STEREORECOGNITION_VPFHEXTRACTOR_H
#define STEREORECOGNITION_VPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/features/vfh.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    class VPFHExtractor : public FeatureExtractor<GlobalDescriptorsPtr> {

    public:

        GlobalDescriptorsPtr run(const PointCloudPtr& points, const SurfaceNormalsPtr& normals);

    };

}


#endif //STEREORECOGNITION_VPFHEXTRACTOR_H

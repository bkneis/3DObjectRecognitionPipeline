#ifndef STEREORECOGNITION_SURFACENORMALESTIMATOR_H
#define STEREORECOGNITION_SURFACENORMALESTIMATOR_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>

namespace preprocessor {

    class SurfaceNormalEstimator {

    public:

        SurfaceNormalsPtr run(const PointCloudPtr& input, float radius);

    };

}


#endif //STEREORECOGNITION_SURFACENORMALESTIMATOR_H

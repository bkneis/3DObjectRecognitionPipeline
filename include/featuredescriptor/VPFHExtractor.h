#ifndef STEREORECOGNITION_VPFHEXTRACTOR_H
#define STEREORECOGNITION_VPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/features/vfh.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    struct VPFHParameters {
        PointCloudPtr points;
        SurfaceNormalsPtr normals;
    };

    class VPFHExtractor : public FeatureExtractor<GlobalDescriptorsPtr, VPFHParameters> {

    public:

        GlobalDescriptorsPtr run(VPFHParameters params)
        {
            pcl::console::print_info ("Extracting features using Viewpoint Feature Histogram \n");
            pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
            vfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            vfh_estimation.setInputCloud (params.points);
            vfh_estimation.setInputNormals (params.normals);
            GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
            vfh_estimation.compute (*global_descriptor);

            return (global_descriptor);
        }


    };

}

#endif //STEREORECOGNITION_VPFHEXTRACTOR_H

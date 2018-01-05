#ifndef STEREORECOGNITION_VPFHEXTRACTOR_H
#define STEREORECOGNITION_VPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/features/vfh.h>
#include "FeatureExtractor.h"

namespace featuredescriptor {

    class VPFHExtractor : public FeatureExtractor<GlobalDescriptorsPtr> {

    public:

        GlobalDescriptorsPtr run(PointCloudPtr points, SurfaceNormalsPtr normals, Config* conf)
        {
            pcl::console::print_info ("Extracting features using Viewpoint Feature Histogram \n");
            pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
            vfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            vfh_estimation.setInputCloud (points);
            vfh_estimation.setInputNormals (normals);
            GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
            vfh_estimation.compute (*global_descriptor);

            return (global_descriptor);
        }


    };

}

#endif //STEREORECOGNITION_VPFHEXTRACTOR_H

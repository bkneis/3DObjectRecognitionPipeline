#ifndef STEREORECOGNITION_CVPFHEXTRACTOR_H
#define STEREORECOGNITION_CVPFHEXTRACTOR_H

#include <typedefs.h>
#include <pcl/console/print.h>
#include <pcl/features/vfh.h>
#include "FeatureExtractor.h"
#include <pcl/features/cvfh.h>

namespace featuredescriptor {

    struct CVPFHParameters {
        PointCloudPtr points;
        SurfaceNormalsPtr normals;
    };

    class CVPFHExtractor : public FeatureExtractor<GlobalDescriptorsPtr> {

    public:

        GlobalDescriptorsPtr run(void* params)
        {
            auto cvpfhParams = static_cast<CVPFHParameters*>(params);
            pcl::console::print_info ("Extracting features using Clustered Viewpoint Feature Histogram \n");
            pcl::CVFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
            vfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
            vfh_estimation.setInputCloud (cvpfhParams->points);
            vfh_estimation.setInputNormals (cvpfhParams->normals);
            GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
            vfh_estimation.compute (*global_descriptor);

            return (global_descriptor);
        }


    };

}

#endif //STEREORECOGNITION_CVPFHEXTRACTOR_H

#ifndef STEREORECOGNITION_SUBJECT_H
#define STEREORECOGNITION_SUBJECT_H

#include <typedefs.h>

namespace classifier {

    template<class FeatureT>
    class Subject {

        typedef typename pcl::PointCloud<FeatureT>::Ptr FeaturePtr;

    public:

        Subject(std::string _name, FeaturePtr _descriptor)
        {
            name = _name;
            descriptor = _descriptor;
        }

        std::string name;
        FeaturePtr descriptor;

    };

}

#endif //STEREORECOGNITION_SUBJECT_H

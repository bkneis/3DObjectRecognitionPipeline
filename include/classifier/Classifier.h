#ifndef STEREORECOGNITION_CLASSIFIER_H
#define STEREORECOGNITION_CLASSIFIER_H

#include <vector>
#include <string>

namespace classifier {

    template <class PointCloudType>
    class Classifier {

    public:
        virtual void classify(PointCloudType subject) = 0;
        virtual void train(const std::vector<std::string> & filenames) = 0;

    };

}

#endif //STEREORECOGNITION_CLASSIFIER_H

#ifndef STEREORECOGNITION_CLASSIFIER_H
#define STEREORECOGNITION_CLASSIFIER_H

#include <vector>
#include <string>
#include "Subject.h"

namespace classifier {

    template <class FeatureT, class FeatureDescriptorPtr>
    class IClassifier {

    public:
        virtual Subject<FeatureT>* classify(FeatureDescriptorPtr subject) = 0;
        virtual void train(const std::vector<std::string> & filenames) = 0;
        virtual void loadModel(std::string filepath) = 0;
        virtual void populateDatabase(std::vector<Subject<GlobalDescriptorT>*> models) = 0;

    };

}

#endif //STEREORECOGNITION_CLASSIFIER_H

#ifndef STEREORECOGNITION_SVM_H
#define STEREORECOGNITION_SVM_H

#include <opencv2/core/mat.hpp>
#include <opencv/ml.h>
#include "IClassifier.h"

#define NTRAINING_SAMPLES   100
#define FRAC_LINEAR_SEP     0.9f

namespace classifier {

    typedef struct SvmData {
        cv::Mat train;
        cv::Mat labels;
    } SvmData;

    template <class FeatureT, class FeatureDescriptor, class FeatureDescriptorPtr>
    class Svm: public IClassifier<FeatureT, FeatureDescriptorPtr> {

    public:

        Svm()
        {
            svm = cv::ml::SVM::create();
        }

        Subject<FeatureT>* classify(FeatureDescriptorPtr subject) override {
            cv::Mat responses(1, 308, CV_32F, subject->points[0].histogram);
            float res = svm->predict(responses);
            std::cout << "res " << res << std::endl;
            if (res > subjects.size()) {
                std::cout << "ERR" << std::endl;
            }
            return subjects.at(res);
        }

        void train(const std::vector<std::string> & filenames) override {}

        void loadModel(std::string filepath) override {}

        void
        populateDatabase(std::vector<classifier::Subject<FeatureT>*> models) override
        {
            subjects = models;
            int numModels = static_cast<int>(models.size());
            float descriptors[numModels][308];
            int labels_[numModels][1];

            for (int i = 0; i < numModels; i++) {
                memcpy(descriptors[i], models.at(i)->descriptor->points[0].histogram, sizeof(float) * numModels);
                int l[] = {i};
                memcpy(labels_[i], l, sizeof(int));
            }

            cv::Mat trainData(static_cast<int>(models.size()), 308, CV_32FC1, descriptors);
            cv::Mat labels   (static_cast<int>(models.size()), 1, CV_32SC1, labels_);

            svm->setGamma(0.50625);
            svm->setC(12.5);
            svm->setKernel(cv::ml::SVM::RBF);
            svm->setType(cv::ml::SVM::C_SVC);
            cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainData, cv::ml::ROW_SAMPLE, labels);
            // bool trained = svm->trainAuto(td, 2);
            bool trained = svm->train(td);
            if (trained) {
                svm->save("/tmp/svm.yml");
            } else {
                std::cout << "ERROR: Could not train the SVM";
            }
        }

    protected:

        SvmData
        convertSubjectsToMat(std::vector<classifier::Subject<FeatureT>*> models)
        {
            SvmData data;
            // todo change 308 to Subject member variable
            cv::Mat trainMat(static_cast<int>(models.size()), 308, CV_32F);
            cv::Mat labels(static_cast<int>(models.size()), 1, CV_32S);
            std::cout << "casted" << static_cast<int>(models.size()) << "   normal " << models.size() << std::endl;
            for(int i = 0; i < trainMat.rows - 1; i++) {
                labels.at<double>(i, 0) = i;
                for(int j = 0; j < trainMat.cols; j++) {
                    trainMat.at<double>(i, j) = models.at(i)->descriptor->points[0].histogram[j];
                }
            }

            data.train = trainMat;
            data.labels = labels;
            return data;
        }

        cv::Ptr<cv::ml::SVM> svm;

    protected:

        std::vector<classifier::Subject<FeatureT>*> subjects;

    };

}

#endif //STEREORECOGNITION_SVM_H

#ifndef STEREORECOGNITION_SVM_H
#define STEREORECOGNITION_SVM_H

#include <opencv2/core/mat.hpp>
#include <opencv/ml.h>
#include "IClassifier.h"

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
            cv::Mat responses(1, 308, CV_32F);
            for(int j = 0; j < responses.cols; j++) {
                responses.at<double>(0, j) = subject->points[0].histogram[j];
            }
            float res = svm->predict(responses, responses);
        }

        void train(const std::vector<std::string> & filenames) override {}

        void loadModel(std::string filepath) override {}

        void
        populateDatabase(std::vector<classifier::Subject<FeatureT>*> models) override
        {
            auto data = convertSubjectsToMat(models);

            svm->setGamma(0.50625);
            svm->setC(12.5);
            svm->setKernel(cv::ml::SVM::RBF);
            svm->setType(cv::ml::SVM::C_SVC);
            cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(data.train, cv::ml::ROW_SAMPLE, data.labels);
            svm->train(td);
            // svm->trainAuto(td);
            svm->save("/tmp/svm.yml");
            // svm->predict(testMat, testResponse);
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

    private:



    };

}

#endif //STEREORECOGNITION_SVM_H

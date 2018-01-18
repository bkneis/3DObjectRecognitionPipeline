#ifndef STEREORECOGNITION_NORMALBAYES_H
#define STEREORECOGNITION_NORMALBAYES_H

#include <opencv2/core/mat.hpp>
#include <opencv/ml.h>
#include "IClassifier.h"
#include "Subject.h"
#include "Svm.h"

#define NTRAINING_SAMPLES   100
#define FRAC_LINEAR_SEP     0.9f

namespace classifier {

    template <class FeatureT, class FeatureDescriptor, class FeatureDescriptorPtr>
    class NormalBayes: public IClassifier<FeatureT, FeatureDescriptorPtr> {

    public:

        NormalBayes()
        {
            nbc = cv::ml::NormalBayesClassifier::create();
        }

        Subject<FeatureT>* classify(FeatureDescriptorPtr subject) override {
            cv::Mat responses(1, 308, CV_32F, subject->points[0].histogram);
            float res = nbc->predict(responses);
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

            nbc->train(trainData, cv::ml::ROW_SAMPLE, labels);
        }

        void
        populateDatabase_(std::vector<classifier::Subject<FeatureT>*> models)
        {
            // auto data = convertSubjectsToMat(models);
            cv::RNG rng(100);
            cv::Mat trainMat(static_cast<int>(models.size()), 308, CV_32FC1);
            cv::Mat labels(static_cast<int>(models.size()), 1, CV_32S);

//            for(int i = 0; i < trainMat.rows; i++) {
//                labels.at<int>(i, 0) = i;
//                for(int j = 0; j < trainMat.cols; j++) {
//                    std::cout << "i and j " << i << "  " << j << std::endl;
//                    trainMat.at<double>(i, j) = models.at(i)->descriptor->points[0].histogram[j];
//                }
//            }

            // cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainMat, cv::ml::ROW_SAMPLE, labels);
            // cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(data.train, cv::ml::ROW_SAMPLE, data.labels);
            // nbc->train(td);
            nbc->train(trainMat, cv::ml::ROW_SAMPLE, labels);
            // nbc->train(data.train, cv::ml::ROW_SAMPLE, data.labels);
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
            for(int i = 0; i < trainMat.rows; i++) {
                labels.at<double>(i, 0) = i;
                for(int j = 0; j < trainMat.cols; j++) {
                    std::cout << models.at(i)->descriptor->points[0].histogram[j] << std::endl;
                    trainMat.at<double>(i, j) = models.at(i)->descriptor->points[0].histogram[j];
                }
            }

            data.train = trainMat;
            data.labels = labels;
            return data;
        }

        cv::Ptr<cv::ml::NormalBayesClassifier> nbc;
        std::vector<classifier::Subject<FeatureT>*> subjects;

    };

}

#endif //STEREORECOGNITION_NORMALBAYES_H

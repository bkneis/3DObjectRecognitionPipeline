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
            cv::Mat responses(1, 308, CV_32F);
            for(int j = 0; j < responses.cols; j++) {
                responses.at<double>(0, j) = subject->points[0].histogram[j];
            }
            float res = nbc->predict(responses);
        }

        void train(const std::vector<std::string> & filenames) override {}

        void loadModel(std::string filepath) override {}

        void
        populateDatabase(std::vector<classifier::Subject<FeatureT>*> models) override
        {
            const int WIDTH = 512, HEIGHT = 512;
            //--------------------- 1. Set up training data randomly ---------------------------------------
            cv::Mat trainData(2*NTRAINING_SAMPLES, 308, CV_32FC1);
            cv::Mat labels   (2*NTRAINING_SAMPLES, 1, CV_32SC1);
            cv::RNG rng(100); // Random value generation class
            // Set up the linearly separable part of the training data
            int nLinearSamples = (int) (FRAC_LINEAR_SEP * NTRAINING_SAMPLES);
            // Generate random points for the class 1
            cv::Mat trainClass = trainData.rowRange(0, nLinearSamples);
            // The x coordinate of the points is in [0, 0.4)
            cv::Mat c = trainClass.colRange(0, 1);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(1), cv::Scalar(0.4 * WIDTH));
            // The y coordinate of the points is in [0, 1)
            c = trainClass.colRange(1,2);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(1), cv::Scalar(HEIGHT));
            // Generate random points for the class 2
            trainClass = trainData.rowRange(2*NTRAINING_SAMPLES-nLinearSamples, 2*NTRAINING_SAMPLES);
            // The x coordinate of the points is in [0.6, 1]
            c = trainClass.colRange(0 , 1);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(0.6*WIDTH), cv::Scalar(WIDTH));
            // The y coordinate of the points is in [0, 1)
            c = trainClass.colRange(1,2);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(1), cv::Scalar(HEIGHT));
            //------------------ Set up the non-linearly separable part of the training data ---------------
            // Generate random points for the classes 1 and 2
            trainClass = trainData.rowRange(  nLinearSamples, 2*NTRAINING_SAMPLES-nLinearSamples);
            // The x coordinate of the points is in [0.4, 0.6)
            c = trainClass.colRange(0,1);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(0.4*WIDTH), cv::Scalar(0.6*WIDTH));
            // The y coordinate of the points is in [0, 1)
            c = trainClass.colRange(1,2);
            rng.fill(c, cv::RNG::UNIFORM, cv::Scalar(1), cv::Scalar(HEIGHT));
            //------------------------- Set up the labels for the classes ---------------------------------
            labels.rowRange(                0,   NTRAINING_SAMPLES).setTo(1);  // Class 1
            labels.rowRange(NTRAINING_SAMPLES, 2*NTRAINING_SAMPLES).setTo(2);  // Class 2

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

    };

}

#endif //STEREORECOGNITION_NORMALBAYES_H

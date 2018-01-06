#ifndef STEREORECOGNITION_KNN_H
#define STEREORECOGNITION_KNN_H

#include <typedefs.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <utils.h>
#include "IClassifier.h"
#include "SubjectNotFound.h"


namespace classifier {

    template <class FeatureT, class FeatureDescriptor, class FeatureDescriptorPtr>
    class KNN : public IClassifier<FeatureT, FeatureDescriptorPtr> {

    public:

        KNN() {}

        Subject<FeatureT>*
        classify(FeatureDescriptorPtr subject) override
        {
            const FeatureT & query_descriptor = subject->points[0];

            std::vector<int> nn_index (1);
            std::vector<float> nn_sqr_distance (1);
            int j = kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
            int i = nn_index[0];
            const int & best_match = nn_index[0];

            if (best_match > models_.size()) {
                throw SubjectNotFound("The nearest neighbour index is out of bounds so the classifier has failed");
            }

            return (models_[best_match]);
        }

        void
        populateDatabase(std::vector<Subject<FeatureT>*> models) override
        {
            models_ = models;
            size_t n = models_.size ();
            descriptors_ = FeatureDescriptorPtr (new FeatureDescriptor);
            for (size_t i = 0; i < n; ++i) {
                *descriptors_ += *(models_[i]->descriptor);
            }
            kdtree_ = typename pcl::KdTreeFLANN<FeatureT>::Ptr (new pcl::KdTreeFLANN<FeatureT>);
            kdtree_->setInputCloud (descriptors_);
        }

        void train(const std::vector<std::string> & filenames) override {}
        void loadModel(const std::string filepath) override {}

    private:

        std::vector<Subject<FeatureT>*> models_;
        FeatureDescriptorPtr descriptors_;
        typename pcl::KdTreeFLANN<FeatureT>::Ptr kdtree_;

    };

}

#endif //STEREORECOGNITION_KNN_H

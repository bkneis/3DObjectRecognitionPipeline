#ifndef STEREORECOGNITION_KNN_H
#define STEREORECOGNITION_KNN_H

#include <typedefs.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <utils.h>
#include "Classifier.h"


namespace classifier {

    class KNN : public Classifier<GlobalDescriptorsPtr> {

    public:

        KNN() : Classifier() {}

        GlobalDescriptorsPtr
        classify(GlobalDescriptorsPtr subject) override
        {
            const GlobalDescriptorT & query_descriptor = subject->points[0];

            std::vector<int> nn_index (1);
            std::vector<float> nn_sqr_distance (1);
            int j = kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
            int i = nn_index[0];
            const int & best_match = nn_index[0];

            auto test = (models_[best_match]);

            return test;
        }

        void
        populateDatabase(std::vector<GlobalDescriptorsPtr> models) override
        {
            models_ = models;
            size_t n = models_.size ();
            descriptors_ = GlobalDescriptorsPtr (new GlobalDescriptors);
            for (size_t i = 0; i < n; ++i) {
                *descriptors_ += *(models_[i]);
            }
            kdtree_ = pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr (new pcl::KdTreeFLANN<GlobalDescriptorT>);
            kdtree_->setInputCloud (descriptors_);
        }

        void train(const std::vector<std::string> & filenames) override {}
        void loadModel(const std::string filepath) override {}

    private:

        std::vector<GlobalDescriptorsPtr> models_;
        GlobalDescriptorsPtr descriptors_;
        pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr kdtree_;

    };

}

#endif //STEREORECOGNITION_KNN_H

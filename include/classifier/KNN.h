#ifndef STEREORECOGNITION_KNN_H
#define STEREORECOGNITION_KNN_H

#include <typedefs.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include "Classifier.h"

namespace classifier {

    class KNN : public Classifier<GlobalDescriptorsPtr> {

    public:

        KNN() : Classifier() {}

        GlobalDescriptorsPtr
        classify(GlobalDescriptorsPtr subject)
        {
            const GlobalDescriptorT & query_descriptor = subject->points[0];

            std::vector<int> nn_index (1);
            std::vector<float> nn_sqr_distance (1);
            kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
            const int & best_match = nn_index[0];

            return (models_[best_match]);
        }

        void
        populateDatabase(std::vector<GlobalDescriptorsPtr> global_descriptors)
        {
            models_ = global_descriptors;
            size_t n = global_descriptors.size ();
            descriptors_ = GlobalDescriptorsPtr (new GlobalDescriptors);
            for (size_t i = 0; i < n; ++i) {
                *descriptors_ += *(global_descriptors[i]);
            }
            kdtree_ = pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr (new pcl::KdTreeFLANN<GlobalDescriptorT>);
            kdtree_->setInputCloud (descriptors_);
        }

        void train(const std::vector<std::string> & filenames) {}
        void loadModel(const std::string filepath) {}

    private:

        std::vector<GlobalDescriptorsPtr> models_;
        GlobalDescriptorsPtr descriptors_;
        pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr kdtree_;

    };

}

#endif //STEREORECOGNITION_KNN_H

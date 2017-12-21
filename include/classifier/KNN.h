#ifndef STEREORECOGNITION_KNN_H
#define STEREORECOGNITION_KNN_H

#include <typedefs.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <utils.h>
#include "Classifier.h"


namespace classifier {

    class KNN : public Classifier<LocalDescriptorsPtr> {

    public:

        KNN() : Classifier() {}

        LocalDescriptorsPtr
        classify(LocalDescriptorsPtr subject)
        {
            const LocalDescriptorT & query_descriptor = subject->points[0];

            std::vector<int> nn_index (1);
            std::vector<float> nn_sqr_distance (1);
            int j = kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
            int i = nn_index[0];
            std::cout << "i is " << i << std::endl;
            const int & best_match = nn_index[0];

            auto test = (models_[best_match]);

            return test;
        }

        void
        populateDatabase(std::string clouds)
        {
            boost::filesystem::path targetDir(clouds);

            boost::filesystem::directory_iterator it(targetDir), eod;

            BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
                if(boost::filesystem::is_regular_file(p)) {
                    auto cloud = loadPointCloud<LocalDescriptorT>(p.string());
                    models_.push_back(cloud);
                }
            }

            size_t n = models_.size ();
            descriptors_ = LocalDescriptorsPtr (new LocalDescriptors);
            for (size_t i = 0; i < n; ++i) {
                *descriptors_ += *(models_[i]);
            }
            kdtree_ = pcl::KdTreeFLANN<LocalDescriptorT>::Ptr (new pcl::KdTreeFLANN<LocalDescriptorT>);
            kdtree_->setInputCloud (descriptors_);
        }

        void train(const std::vector<std::string> & filenames) {}
        void loadModel(const std::string filepath) {}

    private:

        std::vector<LocalDescriptorsPtr> models_;
        LocalDescriptorsPtr descriptors_;
        pcl::KdTreeFLANN<LocalDescriptorT>::Ptr kdtree_;

    };

}

#endif //STEREORECOGNITION_KNN_H

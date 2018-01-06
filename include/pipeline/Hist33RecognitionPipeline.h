#ifndef STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H

#include <featuredescriptor/FPFHExtractor.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <utils.h>
#include <classifier/KNN.h>
#include <classifier/SubjectNotFound.h>
#include "RecognitionPipeline.h"

namespace pipeline {

    template<class PointCloudType>
    class Hist33RecognitionPipeline: public RecognitionPipeline<PointCloudType> {

    public:

        explicit Hist33RecognitionPipeline(Config* conf)
                : RecognitionPipeline<PointCloudType>(conf)
        {
            if (!conf->getClassificationStartegy().compare(knn)) {
                classifier = new classifier::KNN<LocalDescriptorT, LocalDescriptors, LocalDescriptorsPtr>();
            } else {
                classifier = new classifier::KNN<LocalDescriptorT, LocalDescriptors, LocalDescriptorsPtr>();
            }
            auto models = describeDatabase("../data/random");
            classifier->populateDatabase(models);
        }

        void
        visualize() override
        {
            pcl::console::print_info ("Starting visualizer... Close window to exit\n");
            pcl::visualization::PCLVisualizer vis;
            pcl::visualization::PCLHistogramVisualizer hist_vis;
            vis.addPointCloud (RecognitionPipeline<PointCloudType>::input);

            vis.addPointCloudNormals<PointT,NormalT> (RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, 4, 0.02, "normals");

            hist_vis.addFeatureHistogram (*descriptors, 33, "Local descriptor");
            vis.resetCamera ();
            vis.spin();
        }

        void
        describe() override
        {
            // Add keypoints here
            auto extractor = new featuredescriptor::FPFHExtractor();
            descriptors = extractor->run(RecognitionPipeline<PointCloudType>::input, RecognitionPipeline<PointCloudType>::normals, RecognitionPipeline<PointCloudType>::config);
        }

        void
        classify() override
        {
            try {
                auto res = classifier->classify(descriptors);
                std::cout << "The subject is " << res->name << std::endl;
            }
            catch (classifier::SubjectNotFound &ex) {
                std::cout << ex.what() << std::endl;
            }
        }

    protected:

        std::vector<classifier::Subject<LocalDescriptorT>*>
        describeDatabase(std::string clouds)
        {
            std::vector<classifier::Subject<LocalDescriptorT>*> models;
            boost::filesystem::path targetDir(clouds);

            boost::filesystem::directory_iterator it(targetDir), eod;

            BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
                if(boost::filesystem::is_regular_file(p)) {
                    auto cloud = loadPointCloud<PointT>(p.string());
                    this->input = cloud;
                    this->estimateSurfaceNormals();
                    describe();
                    auto subject = new classifier::Subject<LocalDescriptorT>(p.stem().string(), descriptors);
                    models.push_back(subject);
                }
            }

            return models;
        }

        LocalDescriptorsPtr descriptors;
        classifier::IClassifier<LocalDescriptorT, LocalDescriptorsPtr>* classifier;

    };

}


#endif //STEREORECOGNITION_33HISTRECOGNITIONPIPELINE_H

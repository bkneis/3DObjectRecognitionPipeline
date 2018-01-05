#ifndef STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <featuredescriptor/VPFHExtractor.h>
#include <featuredescriptor/CVPFHExtractor.h>
#include <classifier/KNN.h>
#include "RecognitionPipeline.h"

namespace pipeline {

    // todo pass point cloud type and create pointer
    template<class PointCloudType>
    class Hist308RecognitionPipeline : public RecognitionPipeline<PointCloudType> {

    public:

        explicit Hist308RecognitionPipeline(Config *conf)
                : RecognitionPipeline<PointCloudType>(conf)
        {
            if (!conf->getClassificationStartegy().compare(knn)) {
                classifier = new classifier::KNN<GlobalDescriptorT, GlobalDescriptors, GlobalDescriptorsPtr>();
            } else {
                classifier = new classifier::KNN<GlobalDescriptorT, GlobalDescriptors, GlobalDescriptorsPtr>();
            }
            auto models = describeDatabase("../data/random");
            classifier->populateDatabase(models);
        }

        void
        visualize() override
        {
            pcl::console::print_info("Starting visualizer... Close window to exit\n");
            pcl::visualization::PCLVisualizer vis;
            pcl::visualization::PCLHistogramVisualizer hist_vis;
            vis.addPointCloud(this->input);

            vis.addPointCloudNormals<PointT, NormalT>(this->input, this->normals, 4, 0.02, "normals");

            hist_vis.addFeatureHistogram(*descriptors, 308, "Global descriptor");
            vis.resetCamera();
            vis.spin();
        }

        void
        describe() override
        {
            // Configure the feature extractor
            if (!this->config->getFeatureDescriptorStrategy().compare(VPFH)) {
                auto extractor = new featuredescriptor::VPFHExtractor();
                descriptors = extractor->run(this->input, this->normals, this->config);
            }
            else if (!this->config->getFeatureDescriptorStrategy().compare(CVPFH)) {
                auto extractor = new featuredescriptor::CVPFHExtractor();
                descriptors = extractor->run(this->input, this->normals, this->config);
            }
            else {
                // Fall back to default
                auto extractor = new featuredescriptor::VPFHExtractor();
                descriptors = extractor->run(this->input, this->normals, this->config);
            }
        }

        void
        classify() override
        {
            auto res = classifier->classify(descriptors);
            std::cout << "The subject is " << res->name << std::endl;
        }

    protected:

        std::vector<classifier::Subject<GlobalDescriptorT>*>
        describeDatabase(std::string clouds)
        {
            std::vector<classifier::Subject<GlobalDescriptorT>*> models;
            boost::filesystem::path targetDir(clouds);

            boost::filesystem::directory_iterator it(targetDir), eod;

            BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
                if(boost::filesystem::is_regular_file(p)) {
                    auto cloud = loadPointCloud<PointT>(p.string());
                    this->input = cloud;
                    this->estimateSurfaceNormals();
                    describe();
                    auto subject = new classifier::Subject<GlobalDescriptorT>(p.stem().string(), descriptors);
                    models.push_back(subject);
                }
            }

            return models;
        }

        GlobalDescriptorsPtr descriptors;
        classifier::IClassifier<GlobalDescriptorT, GlobalDescriptorsPtr>* classifier;

    };
}

#endif //STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
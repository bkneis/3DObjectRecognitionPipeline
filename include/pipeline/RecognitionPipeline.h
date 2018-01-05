#ifndef STEREORECOGNITION_RECOGNITIONPIPELINE_H
#define STEREORECOGNITION_RECOGNITIONPIPELINE_H

#include <chrono>
#include <preprocessor/filters.h>
#include <preprocessor/SurfaceNormalEstimator.h>
#include "Config.h"
#include "configdefs.h"

namespace pipeline {

    template<class PointCloudType>
    class RecognitionPipeline {

    public:

        virtual void visualize() = 0;

        virtual void describe() = 0;

        virtual void classify() = 0;

        void
        estimateSurfaceNormals()
        {
            // Configure the normal estimation strategy
            if (!config->getNormalsStrategy().compare(APPROXIMATIONS)) {
                auto estimator = new preprocessor::SurfaceNormalEstimator<PointCloudType>();
                normals = estimator->run(input, config);
            }
            else {
                // Fall back to default
                auto estimator = new preprocessor::SurfaceNormalEstimator<PointCloudType>();
                normals = estimator->run(input, config);
            }
        }

        void
        run(PointCloudType _input)
        {
            // Perform segmentation and remove background
            input = preprocessor::removeOutliers(_input, 0.3, 300);

            auto t1 = std::chrono::high_resolution_clock::now();

            estimateSurfaceNormals();
            describe();
            classify();
            visualize();

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            std::cout << "Pipeline took " << duration << " micro seconds" << std::endl;
        }

    protected:

        explicit RecognitionPipeline(Config* conf)
        {
            config = conf;
        }

        PointCloudPtr input;
        SurfaceNormalsPtr normals;
        Config* config;

    };

}


#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H
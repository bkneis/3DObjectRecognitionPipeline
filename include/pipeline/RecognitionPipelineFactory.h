#ifndef STEREORECOGNITION_RECOGNITIONPIPELINEFACTORY_H
#define STEREORECOGNITION_RECOGNITIONPIPELINEFACTORY_H

#include "RecognitionPipeline.h"
#include "Config.h"
#include "Hist33RecognitionPipeline.h"
#include "Hist308RecognitionPipeline.h"

namespace pipeline {

    template<class PointCloudType>
    class RecognitionPipelineFactory {

    public:

        static RecognitionPipeline<PointCloudType>*
        create(Config* conf)
        {
            if (!conf->getFeatureDescriptorStrategy().compare(FPFH)) {
                return new Hist33RecognitionPipeline<PointCloudType>(conf);
            }

            return new Hist308RecognitionPipeline<PointCloudType>(conf);
        }

    };

}


#endif //STEREORECOGNITION_RECOGNITIONPIPELINEFACTORY_H

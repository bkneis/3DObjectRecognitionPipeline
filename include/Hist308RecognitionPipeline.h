#ifndef STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
#define STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H

#include "RecognitionPipeline.h"

namespace pipeline {

    class Hist308RecognitionPipeline: public RecognitionPipeline {

    public:

        explicit Hist308RecognitionPipeline(Config* conf)
        {
            // Initialise classifier and populate database
        }

        void
        visualize() override {

        }

        void
        describe() override {

        }

        void
        classify() override {

        }

    };
}

#endif //STEREORECOGNITION_308HISTRECOGNITIONPIPELINE_H
#ifndef STEREORECOGNITION_RECOGNITIONPIPELINE_H
#define STEREORECOGNITION_RECOGNITIONPIPELINE_H

#include <vector>
#include "Node.h"
#include "preprocessor/SIFTKeyPointDetector.h"

class RecognitionPipeline {

public:

    void run();
    void add(Node* keypointDetector);

private:

    std::vector<Node*> nodes;

};


#endif //STEREORECOGNITION_RECOGNITIONPIPELINE_H

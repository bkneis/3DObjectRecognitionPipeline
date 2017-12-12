#ifndef STEREORECOGNITION_CCFACEDETECTOR_H
#define STEREORECOGNITION_CCFACEDETECTOR_H

#include <iostream>

class CCFaceDetector {

public:
    bool detect(std::string file_path);

private:
    bool detectCPU(std::string file_path);
    bool detectGPU(std::string file_path);

};


#endif //STEREORECOGNITION_CCFACEDETECTOR_H

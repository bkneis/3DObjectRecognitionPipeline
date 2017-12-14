#ifndef STEREORECOGNITION_CCFACEDETECTOR_H
#define STEREORECOGNITION_CCFACEDETECTOR_H

#include <iostream>
#include <opencv2/core/mat.hpp>

class CCFaceDetector {

public:
    bool detect(FlyCapture2::Image image);

private:
    cv::Mat convertFlyToCv(FlyCapture2::Image raw_image);
    bool detectCPU(cv::Mat image);
    bool detectGPU(cv::Mat image);

};


#endif //STEREORECOGNITION_CCFACEDETECTOR_H

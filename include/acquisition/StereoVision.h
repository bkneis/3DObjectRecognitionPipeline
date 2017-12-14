#ifndef STEREORECOGNITION_STEREOVISION_H
#define STEREORECOGNITION_STEREOVISION_H

#include "FlyCapture2.h"

#define OUTPUT_FILE_PATH "/tmp/"
#define OUTPUT_FILE_TYPE ".png"

namespace acquisition {

    class StereoVision {

    public:
        int run();

    private:
        void PrintCameraInfo(FlyCapture2::CameraInfo *pCamInfo);
        int setupCameras();
        FlyCapture2::Camera* cameras;
        FlyCapture2::CameraInfo* camerasInfo;
    };

}

#endif //STEREORECOGNITION_STEREOVISION_H

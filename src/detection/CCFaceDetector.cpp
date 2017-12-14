#include <Image.h>
#include "detection/CCFaceDetector.h"
#include "opencv2/objdetect.hpp"
#include "opencv2/cudaobjdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

#define USING_GPU true // change this to read from Cmake if gpu is supported

bool
CCFaceDetector::detect(FlyCapture2::Image image)
{
    Mat cvImage = convertFlyToCv(image);
    return USING_GPU ? detectGPU(cvImage) : detectCPU(cvImage);
}

bool
CCFaceDetector::detectCPU(Mat image)
{
    CascadeClassifier face_cascade;

    if( !face_cascade.load( "/home/arthur/face_cascade.xml" ) ) {
        printf("--(!)Error loading face cascade\n");
        throw "Cannot open face cascade file";
    }

    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor( image, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    for (int i = 0; i < faces.size(); i++) {
        // Remove outliers
        if (faces[i].width < 200 || faces[i].height < 200) {
            faces.erase(faces.begin() + i);
            continue;
        }
        std::cout << "detect face above the threshold" << faces[i].width << " " << faces[i].height << std::endl;
        Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( image, center, Size(faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    if (faces.size() > 0) {
        ostringstream filename;
        filename << "/home/arthur/test" << std::to_string(rand() % 100) << ".png";
        //-- Show what you got
        imwrite(filename.str(), image);
    }

    return faces.size() > 0;
}

bool
CCFaceDetector::detectGPU(Mat image)
{
    Ptr<cuda::CascadeClassifier> cascade_gpu = cuda::CascadeClassifier::create("/home/arthur/face_cascade_cuda.xml");
    cuda::GpuMat image_gpu(image);
    cuda::GpuMat objbuf;
    cascade_gpu->detectMultiScale(image_gpu, objbuf);
    std::vector<Rect> faces;
    cascade_gpu->convert(objbuf, faces);
    for (int i = 0; i < faces.size(); i++) {
        // Remove outliers
        if (faces[i].width < 200 || faces[i].height < 200) {
            faces.erase(faces.begin() + i);
            continue;
        }
        std::cout << "detect face above the threshold" << faces[i].width << " " << faces[i].height << std::endl;
        cv::rectangle(image, faces[i], Scalar(255));
    }

    if (faces.size() > 0) {
        ostringstream filename;
        filename << "/home/arthur/cuda_test" << std::to_string(rand() % 100) << ".png";
        //-- Show what you got
        imwrite(filename.str(), image);
    }

    return faces.size() > 0;
}

cv::Mat CCFaceDetector::convertFlyToCv(FlyCapture2::Image raw_image) {

    FlyCapture2::Image image;
    cv::Mat frame; // temp;

    unsigned int rowBytes = raw_image.GetCols();
    frame = cv::Mat(raw_image.GetRows(), raw_image.GetCols(), CV_8UC1, raw_image.GetData(), rowBytes);

    return frame.clone();
}

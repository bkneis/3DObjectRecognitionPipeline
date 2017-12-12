#include "detection/CCFaceDetector.h"
#include "opencv2/objdetect.hpp"
#include "opencv2/cudaobjdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

bool
CCFaceDetector::detect(std::string path)
{
    return 1 == 1 ? detectCPU(path) : detectCPU(path);
}

bool
CCFaceDetector::detectCPU(std::string path)
{
    CascadeClassifier face_cascade;

    if( !face_cascade.load( "/home/arthur/face_cascade.xml" ) ) {
        printf("--(!)Error loading face cascade\n");
        throw "Cannot open face cascade file";
    }

    Mat frame = imread(path);

    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    for (auto &face : faces) {
        std::cout << "detect face " << face.width << " " << face.height << std::endl;
        Point center(face.x + face.width/2, face.y + face.height/2 );
        ellipse( frame, center, Size(face.width/2, face.height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    if (faces.size() > 0) {
        ostringstream filename;
        filename << "/home/arthur/test" << std::to_string(rand() % 100) << ".png";
        //-- Show what you got
        imwrite(filename.str(), frame);
    }

    return faces.size() > 0;
}

bool
CCFaceDetector::detectGPU(std::string path)
{
    Ptr<cuda::CascadeClassifier> cascade_gpu = cuda::CascadeClassifier::create("/home/arthur/face_cascade_cuda.xml");
    Mat image_cpu = imread(path);
    cuda::GpuMat image_gpu(image_cpu);
    cuda::GpuMat objbuf;
    cascade_gpu->detectMultiScale(image_gpu, objbuf);
    std::vector<Rect> faces;
    cascade_gpu->convert(objbuf, faces);
    for (int i = 0; i < faces.size(); i++) {
        if (faces[i].width < 100 || faces[i].height < 100) {
            faces.erase(faces.begin() + i);
            continue;
        }
        std::cout << "detect face " << faces[i].width << " " << faces[i].height << std::endl;
        cv::rectangle(image_cpu, faces[i], Scalar(255));
    }

    if (faces.size() > 0) {
        ostringstream filename;
        filename << "/home/arthur/cuda_test" << std::to_string(rand() % 100) << ".png";
        //-- Show what you got
        imwrite(filename.str(), image_cpu);
    }

    return faces.size() > 0;
}
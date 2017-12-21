#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <opencv2/cudaobjdetect.hpp>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

void detectAndDisplayCPU( Mat frame );
void detectAndDisplayGPU( Mat frame );

String face_cascade_name, eyes_cascade_name, image;
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
String window_name = "Capture - Face detection";

Ptr<cuda::CascadeClassifier> cascade_gpu = cuda::CascadeClassifier::create("/home/arthur/face_cascade_cuda.xml");

int main( int argc, const char** argv )
{
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                                     "{face_cascade|../../data/haarcascades/haarcascade_frontalface_alt.xml|}"
                                     "{eyes_cascade|../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|}"
                                     "{image|../../data/image.png|}");
    cout << "\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
            "You can use Haar or LBP features.\n\n";
    parser.printMessage();

    face_cascade_name = parser.get<string>("face_cascade");
    eyes_cascade_name = parser.get<string>("eyes_cascade");
    image = parser.get<string>("image");

    Mat frame;
    //-- 1. Load the cascades
    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
    if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };

    frame = imread(image);

    if( frame.empty() ) {
        printf(" --(!) No captured frame -- Break!");
        return 0;
    }

    detectAndDisplayCPU( frame );

    detectAndDisplayGPU( frame );


    return 0;
}

void detectAndDisplayCPU( Mat frame )
{
    std::vector<Rect> faces;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    face_cascade.detectMultiScale( frame, faces, 1.3, 5, 0|CASCADE_SCALE_IMAGE );
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    cout << "CPU took " << duration << endl;

    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    Mat resized;
    resize(frame, resized, Size(), 0.5, 0.5);

    imshow( window_name, resized );

    while (true) {
        char c = (char)waitKey(10);
        if( c == 27 ) { break; } // press escape
    }

}

void detectAndDisplayGPU( Mat frame )
{
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    cuda::GpuMat image_gpu(frame);
    cuda::GpuMat objbuf;
    cascade_gpu->detectMultiScale(image_gpu, objbuf);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    cout << "GPU took " << duration << endl;

    std::vector<Rect> faces;
    cascade_gpu->convert(objbuf, faces);
    for (int i = 0; i < faces.size(); i++) {
        cv::rectangle(frame, faces[i], Scalar(255));
    }

    Mat resized;
    resize(frame, resized, Size(), 0.5, 0.5);
    imshow( window_name, resized );

    while (true) {
        char c = (char)waitKey(10);
        if( c == 27 ) { break; } // press escape
    }

}
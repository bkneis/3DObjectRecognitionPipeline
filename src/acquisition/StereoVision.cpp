#include "acquisition/StereoVision.h"
#include <iostream>
#include <sstream>
#include <acquisition/CCFaceDetector.h>
#include <reconstruct.h>

using namespace FlyCapture2;
using namespace std;

void
acquisition::StereoVision::PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

int
acquisition::StereoVision::setupCameras()
{
    Error error;

    // Initialize BusManager and retrieve number of cameras detected
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        error.PrintErrorTrace();
        return -1;
    }

    cout << "Number of cameras detected: " << numCameras << endl;

    // Check to make sure at least two cameras are connected before
    // running example
    if (numCameras < 1) {
        cout << "Insufficient number of cameras." << endl;
        cout << "Make sure at least two cameras are connected for example to "
                "run."
             << endl;
        cout << "Press Enter to exit." << endl;
        cin.ignore();
        return -1;
    }

    cameras = new Camera[numCameras];

    camerasInfo = new CameraInfo[numCameras];

    for (unsigned int i = 0; i < numCameras; i++) {
        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }

        // Connect to a camera
        error = cameras[i].Connect(&guid);
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = cameras[i].GetCameraInfo(&camInfo);
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }
        camerasInfo[i] = camInfo;

        PrintCameraInfo(&camInfo);

        // Turn trigger mode off
        TriggerMode trigMode;
        trigMode.onOff = false;
        error = cameras[i].SetTriggerMode(&trigMode);
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }

        // Turn Timestamp on
        EmbeddedImageInfo imageInfo;
        imageInfo.timestamp.onOff = true;
        error = cameras[i].SetEmbeddedImageInfo(&imageInfo);
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }

        // Start streaming on camera
        error = cameras[i].StartCapture();
        if (error != PGRERROR_OK) {
            error.PrintErrorTrace();
            delete[] cameras;
            return -1;
        }
    }
    return 1;
}

int
acquisition::StereoVision::run()
{
    setupCameras();

    Error error;
    // Create an image array to hold 1 image per camera
    Image* images = new Image[1];

    srand (time(NULL));

    auto detector = new CCFaceDetector;

    while (true) {
        for (unsigned int i = 0; i < 1; i++) {
            Image image;
            error = cameras[i].RetrieveBuffer(&image);
            if (error != PGRERROR_OK) {
                error.PrintErrorTrace();
                delete[] cameras;
                return -1;
            }
            images[i] = image;
        }

        bool isFace = detector->detect(images[0]);
//        if (isFace) {
//            std::cout << "face detected" << endl;
//            ostringstream filename;
//            filename << "/tmp/" << std::to_string(rand() % 100) << "1" << OUTPUT_FILE_TYPE;
//            std::string leftPath = filename.str();
//
//            // Save the image. If a file format is not passed in, then the file
//            // extension is parsed to attempt to determine the file format.
//            error = images[0].Save(leftPath.c_str());
//            if (error != PGRERROR_OK) {
//                error.PrintErrorTrace();
//                return -1;
//            }
//
//            filename.clear();
//
//            filename << "/tmp/" << std::to_string(rand() % 100) << "2" << OUTPUT_FILE_TYPE;
//            std::string rightPath = filename.str();
//
//            // Save the image. If a file format is not passed in, then the file
//            // extension is parsed to attempt to determine the file format.
//            error = images[1].Save(rightPath.c_str());
//            if (error != PGRERROR_OK) {
//                error.PrintErrorTrace();
//                return -1;
//            }
//
//            generatePointCloud(leftPath, rightPath);
//        }
    }

}
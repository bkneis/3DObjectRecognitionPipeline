#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#define NTRAINING_SAMPLES   100         // Number of training samples per class
#define FRAC_LINEAR_SEP     0.9f        // Fraction of samples which compose the linear separable part
using namespace cv;
using namespace cv::ml;
using namespace std;
static void help()
{
    cout<< "\n--------------------------------------------------------------------------" << endl
        << "This program shows Support Vector Machines for Non-Linearly Separable Data. " << endl
        << "Usage:"                                                               << endl
        << "./non_linear_svms" << endl
        << "--------------------------------------------------------------------------"   << endl
        << endl;
}
int main()
{
    help();
    // Data for visual representation
    const int WIDTH = 512, HEIGHT = 512;
    Mat I = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    //--------------------- 1. Set up training data randomly ---------------------------------------
    Mat trainData(2*NTRAINING_SAMPLES, 308, CV_32FC1);
    Mat labels   (2*NTRAINING_SAMPLES, 1, CV_32SC1);
    RNG rng(100); // Random value generation class
    // Set up the linearly separable part of the training data
    int nLinearSamples = (int) (FRAC_LINEAR_SEP * NTRAINING_SAMPLES);
    // Generate random points for the class 1
    Mat trainClass = trainData.rowRange(0, nLinearSamples);
    // The x coordinate of the points is in [0, 0.4)
    Mat c = trainClass.colRange(0, 1);
    rng.fill(c, RNG::UNIFORM, Scalar(1), Scalar(0.4 * WIDTH));
    // The y coordinate of the points is in [0, 1)
    c = trainClass.colRange(1,2);
    rng.fill(c, RNG::UNIFORM, Scalar(1), Scalar(HEIGHT));
    // Generate random points for the class 2
    trainClass = trainData.rowRange(2*NTRAINING_SAMPLES-nLinearSamples, 2*NTRAINING_SAMPLES);
    // The x coordinate of the points is in [0.6, 1]
    c = trainClass.colRange(0 , 1);
    rng.fill(c, RNG::UNIFORM, Scalar(0.6*WIDTH), Scalar(WIDTH));
    // The y coordinate of the points is in [0, 1)
    c = trainClass.colRange(1,2);
    rng.fill(c, RNG::UNIFORM, Scalar(1), Scalar(HEIGHT));
    //------------------ Set up the non-linearly separable part of the training data ---------------
    // Generate random points for the classes 1 and 2
    trainClass = trainData.rowRange(  nLinearSamples, 2*NTRAINING_SAMPLES-nLinearSamples);
    // The x coordinate of the points is in [0.4, 0.6)
    c = trainClass.colRange(0,1);
    rng.fill(c, RNG::UNIFORM, Scalar(0.4*WIDTH), Scalar(0.6*WIDTH));
    // The y coordinate of the points is in [0, 1)
    c = trainClass.colRange(1,2);
    rng.fill(c, RNG::UNIFORM, Scalar(1), Scalar(HEIGHT));
    //------------------------- Set up the labels for the classes ---------------------------------
    labels.rowRange(                0,   NTRAINING_SAMPLES).setTo(1);  // Class 1
    labels.rowRange(NTRAINING_SAMPLES, 2*NTRAINING_SAMPLES).setTo(2);  // Class 2
    //------------------------ 2. Set up the support vector machines parameters --------------------
    //------------------------ 3. Train the svm ----------------------------------------------------
    cout << "Starting training process" << endl;
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setC(0.1);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e7, 1e-6));
    svm->train(trainData, ROW_SAMPLE, labels);
    cout << "Finished training process" << endl;
    //------------------------ 4. Show the decision regions ----------------------------------------
    Vec3b green(0,100,0), blue (100,0,0);
    for (int i = 0; i < I.rows; ++i)
        for (int j = 0; j < I.cols; ++j)
        {
            Mat sampleMat = (Mat_<float>(1,308) << i, j);
            float response = svm->predict(sampleMat);
            if      (response == 1)    I.at<Vec3b>(j, i)  = green;
            else if (response == 2)    I.at<Vec3b>(j, i)  = blue;
        }
    //----------------------- 5. Show the training data --------------------------------------------
    int thick = -1;
    int lineType = 8;
    float px, py;
    // Class 1
    for (int i = 0; i < NTRAINING_SAMPLES; ++i)
    {
        px = trainData.at<float>(i,0);
        py = trainData.at<float>(i,1);
        circle(I, Point( (int) px,  (int) py ), 3, Scalar(0, 255, 0), thick, lineType);
    }
    // Class 2
    for (int i = NTRAINING_SAMPLES; i <2*NTRAINING_SAMPLES; ++i)
    {
        px = trainData.at<float>(i,0);
        py = trainData.at<float>(i,1);
        circle(I, Point( (int) px, (int) py ), 3, Scalar(255, 0, 0), thick, lineType);
    }
    //------------------------- 6. Show support vectors --------------------------------------------
    thick = 2;
    lineType  = 8;
    Mat sv = svm->getUncompressedSupportVectors();
    for (int i = 0; i < sv.rows; ++i)
    {
        const float* v = sv.ptr<float>(i);
        circle( I,  Point( (int) v[0], (int) v[1]), 6, Scalar(128, 128, 128), thick, lineType);
    }
    imwrite("result.png", I);                      // save the Image
    imshow("SVM for Non-Linear Training Data", I); // show it to the user
    waitKey(0);
}



//This code will run in both opencv 2 and 3. Just change the first two macros in the code according to the requirement.

#define USE_OPENCV_3
//#define USE_OPENCV_2


//#include <iostream>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include "opencv2/objdetect.hpp"
//#include <opencv2/ml.hpp>
//
//#include <iostream>
//
//
//using namespace cv::ml;
//using namespace cv;
//using namespace std;
//
//
//
//
//string pathName = "digits.png";
//int SZ = 20;
//float affineFlags = WARP_INVERSE_MAP|INTER_LINEAR;
//
//Mat deskew(Mat& img){
//    Moments m = moments(img);
//    if(abs(m.mu02) < 1e-2){
//        return img.clone();
//    }
//    float skew = m.mu11/m.mu02;
//    Mat warpMat = (Mat_<float>(2,3) << 1, skew, -0.5*SZ*skew, 0, 1, 0);
//    Mat imgOut = Mat::zeros(img.rows, img.cols, img.type());
//    warpAffine(img, imgOut, warpMat, imgOut.size(),affineFlags);
//
//    return imgOut;
//}
//
//void loadTrainTestLabel(string &pathName, vector<Mat> &trainCells, vector<Mat> &testCells,vector<int> &trainLabels, vector<int> &testLabels){
//
//    Mat img = imread(pathName,CV_LOAD_IMAGE_GRAYSCALE);
//    int ImgCount = 0;
//    for(int i = 0; i < img.rows; i = i + SZ)
//    {
//        for(int j = 0; j < img.cols; j = j + SZ)
//        {
//            Mat digitImg = (img.colRange(j,j+SZ).rowRange(i,i+SZ)).clone();
//            if(j < int(0.9*img.cols))
//            {
//                trainCells.push_back(digitImg);
//            }
//            else
//            {
//                testCells.push_back(digitImg);
//            }
//            ImgCount++;
//        }
//    }
//
//    cout << "Image Count : " << ImgCount << endl;
//    float digitClassNumber = 0;
//
//    for(int z=0;z<int(0.9*ImgCount);z++){
//        if(z % 450 == 0 && z != 0){
//            digitClassNumber = digitClassNumber + 1;
//        }
//        trainLabels.push_back(digitClassNumber);
//    }
//    digitClassNumber = 0;
//    for(int z=0;z<int(0.1*ImgCount);z++){
//        if(z % 50 == 0 && z != 0){
//            digitClassNumber = digitClassNumber + 1;
//        }
//        testLabels.push_back(digitClassNumber);
//    }
//}
//
//void CreateDeskewedTrainTest(vector<Mat> &deskewedTrainCells,vector<Mat> &deskewedTestCells, vector<Mat> &trainCells, vector<Mat> &testCells){
//
//
//    for(int i=0;i<trainCells.size();i++){
//
//        Mat deskewedImg = deskew(trainCells[i]);
//        deskewedTrainCells.push_back(deskewedImg);
//    }
//
//    for(int i=0;i<testCells.size();i++){
//
//        Mat deskewedImg = deskew(testCells[i]);
//        deskewedTestCells.push_back(deskewedImg);
//    }
//}
//
//HOGDescriptor hog(
//        Size(20,20), //winSize
//        Size(10,10), //blocksize
//        Size(5,5), //blockStride,
//        Size(10,10), //cellSize,
//        9, //nbins,
//        1, //derivAper,
//        -1, //winSigma,
//        0, //histogramNormType,
//        0.2, //L2HysThresh,
//        0,//gammal correction,
//        64,//nlevels=64
//        1);
//void CreateTrainTestHOG(vector<vector<float> > &trainHOG, vector<vector<float> > &testHOG, vector<Mat> &deskewedtrainCells, vector<Mat> &deskewedtestCells){
//
//    for(int y=0;y<deskewedtrainCells.size();y++){
//        vector<float> descriptors;
//        hog.compute(deskewedtrainCells[y],descriptors);
//        trainHOG.push_back(descriptors);
//    }
//
//    for(int y=0;y<deskewedtestCells.size();y++){
//
//        vector<float> descriptors;
//        hog.compute(deskewedtestCells[y],descriptors);
//        testHOG.push_back(descriptors);
//    }
//}
//void ConvertVectortoMatrix(vector<vector<float> > &trainHOG, vector<vector<float> > &testHOG, Mat &trainMat, Mat &testMat)
//{
//
//    int descriptor_size = trainHOG[0].size();
//
//    for(int i = 0;i<trainHOG.size();i++){
//        for(int j = 0;j<descriptor_size;j++){
//            trainMat.at<float>(i,j) = trainHOG[i][j];
//        }
//    }
//    for(int i = 0;i<testHOG.size();i++){
//        for(int j = 0;j<descriptor_size;j++){
//            testMat.at<float>(i,j) = testHOG[i][j];
//        }
//    }
//}
//
//void getSVMParams(SVM *svm)
//{
//    cout << "Kernel type     : " << svm->getKernelType() << endl;
//    cout << "Type            : " << svm->getType() << endl;
//    cout << "C               : " << svm->getC() << endl;
//    cout << "Degree          : " << svm->getDegree() << endl;
//    cout << "Nu              : " << svm->getNu() << endl;
//    cout << "Gamma           : " << svm->getGamma() << endl;
//}
//
//void SVMtrain(Mat &trainMat,vector<int> &trainLabels, Mat &testResponse,Mat &testMat){
//
//    Ptr<SVM> svm = SVM::create();
//    svm->setGamma(0.50625);
//    svm->setC(12.5);
//    svm->setKernel(SVM::RBF);
//    svm->setType(SVM::C_SVC);
//    Ptr<TrainData> td = TrainData::create(trainMat, ROW_SAMPLE, trainLabels);
//    svm->train(td);
//    //svm->trainAuto(td);
//    svm->save("model4.yml");
//    svm->predict(testMat, testResponse);
//    getSVMParams(svm);
//
//}
//void SVMevaluate(Mat &testResponse,float &count, float &accuracy,vector<int> &testLabels){
//
//    for(int i=0;i<testResponse.rows;i++)
//    {
//        //cout << testResponse.at<float>(i,0) << " " << testLabels[i] << endl;
//        if(testResponse.at<float>(i,0) == testLabels[i]){
//            count = count + 1;
//        }
//    }
//    accuracy = (count/testResponse.rows)*100;
//}
//int main(){
//
//    vector<Mat> trainCells;
//    vector<Mat> testCells;
//    vector<int> trainLabels;
//    vector<int> testLabels;
//    loadTrainTestLabel(pathName,trainCells,testCells,trainLabels,testLabels);
//
//    vector<Mat> deskewedTrainCells;
//    vector<Mat> deskewedTestCells;
//    CreateDeskewedTrainTest(deskewedTrainCells,deskewedTestCells,trainCells,testCells);
//
//    std::vector<std::vector<float> > trainHOG;
//    std::vector<std::vector<float> > testHOG;
//    CreateTrainTestHOG(trainHOG,testHOG,deskewedTrainCells,deskewedTestCells);
//
//    int descriptor_size = trainHOG[0].size();
//    cout << "Descriptor Size : " << descriptor_size << endl;
//
//    Mat trainMat(trainHOG.size(),descriptor_size,CV_32FC1);
//    Mat testMat(testHOG.size(),descriptor_size,CV_32FC1);
//
//    ConvertVectortoMatrix(trainHOG,testHOG,trainMat,testMat);
//
//    Mat testResponse;
//    SVMtrain(trainMat,trainLabels,testResponse,testMat);
//
//
//    float count = 0;
//    float accuracy = 0 ;
//    SVMevaluate(testResponse,count,accuracy,testLabels);
//
//    cout << "Accuracy        : " << accuracy << "%"<< endl;
//    return 0;
//}
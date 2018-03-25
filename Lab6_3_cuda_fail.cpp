//-----------------------------------------------------------------------------------------------------------------
// File:   example_perspectiveandpath.cpp
// Author: Peter J. Tucker 
//
// Modified from Source File:   example_2.cpp
// Original Source File Author: Prof. Allan A. Douglas 
//-----------------------------------------------------------------------------------------------------------------

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <iostream>
#include <string>
#include <algorithm> 
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <utility>
#include "lsf.h"

//#define _CRT_SECURE_NO_WARNINGS

// Select Video Source
// The MP4 demo uses a ROI for better tracking of the moving object
//#define TEST_LIVE_VIDEO

//Adjusting ROI (region of interest)
#define XROI 518    // top-left x-coord
#define YROI 92     // top-left y-coord
#define LEN 310
#define HI 531

// Top Paddle ROI
#define XROIT 518
#define YROIT 10
#define LENT 310
#define HIT 82

// Bottom Paddle ROI
#define XROIB 518
#define YROIB 623
#define LENB 310
#define HIB 82

// Trackbar variables
#define min 0
#define max 256

// Block and Aperture size for finding corners
#define blksz 2  //2
#define aperture 7  //7

// Array length
#define n 10

using namespace cv;
using namespace std;


// Min and Max HSV filter values.
//Entire Field
int HF_MIN = 0;
int HF_MAX = 255;//178;
int SF_MIN = 0;
int SF_MAX = 255;
int VF_MIN = 84;//93;//84;
int VF_MAX = 255;

// Time frame period in ms
double Ts = 33.333;


//-----------------------------------------------------------------------------------------------------------------
// int to string helper function
//-----------------------------------------------------------------------------------------------------------------
string intToString(int number){
 
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}


//-----------------------------------------------------------------------------------------------------------------
// Dialate
//-----------------------------------------------------------------------------------------------------------------
//create structuring element that will be used to "dilate" and "erode" image.
void morphOps(cv::Mat &thresh){

	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));

    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(9,9));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

}


//-----------------------------------------------------------------------------------------------------------------
// Find Field Corners
//-----------------------------------------------------------------------------------------------------------------
void FindCorners(cv::Mat src, cv::Mat &cameraFeed){
        
    // Allocating size of image    
    cv::cuda::GpuMat gpu_src;//(src.size(), CV_32FC1, Scalar::all(0));
    gpu_src.upload(src);

    cv::cuda::GpuMat dst(gpu_src.size(), CV_32FC1, Scalar::all(0));
    cv::cuda::GpuMat dst_norm(gpu_src.size(), CV_32FC1, Scalar::all(0));
/*
    // Allocating size of image    
    cv::Mat dst(src.size(), CV_32FC1, Scalar::all(0));
    cv::Mat dst_norm(src.size(), CV_32FC1, Scalar::all(0));
*/

    
    // Allocating size of image
//    dst = cv::Mat::zeros(src.size(), CV_32FC1);
//    dst_norm = cv::Mat::zeros(src.size(), CV_32FC1);

    int thresh = 210;  //180
    int apertureSize = aperture;
    int blockSize = blksz;
    double harrisK = 0.058;


cv::Ptr<cv::cuda::CornernessCriteria> corners = cv::cuda::createHarrisCorner(CV_32FC1,blockSize,apertureSize,harrisK, BORDER_REFLECT101);

corners->compute(gpu_src, dst);	

cv::Mat out;
dst.download(out);

    // Drawing circles around corners
    for( int j=0; j<out.rows; j++)
    {
        for( int i=0; i< out.cols; i++)
        {
            if( (int) out.at<float>(j,i) > thresh )
            {
                cv::circle(cameraFeed, Point(i,j), 5, Scalar(55,255,55), 2, 8, 0);
                std::cout << "corner points: (" << i << "," << j << ")" << '\n';

            }
        }
    }


/*
    // Detecting corners
    cv::cornerHarris( src, dst, blockSize, apertureSize, harrisK, BORDER_DEFAULT );

    // Normalize
    cv::normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

    // Drawing circles around corners
    for( int j=0; j<dst_norm.rows; j++)
    {
        for( int i=0; i< dst_norm.cols; i++)
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                cv::circle(cameraFeed, Point(i,j), 5, Scalar(55,255,55), 2, 8, 0);
//                std::cout << "fuck you corners: (" << i << "," << j << ")" << '\n';

            }
        }
    }*/
}



//-----------------------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------------------
int main() {


    // OpenCV frame matrices
    cv::Mat frame0, frame1, frame0_warped, frame1_warped, result, HSV_0, HSV_1, thresholdfilter_0, thresholdfilter_1, gray0, gray1, topfilter0, topfilter1, bottomfilter0, bottomfilter1, threshold;

    cv::cuda::GpuMat gpu_frame0, gpu_frame1, gpu_frame0_warped, gpu_frame1_warped, gpu_grayImage0, gpu_grayImage1, gpu_differenceImage, gpu_thresholdImage, gpu_HSV_0, gpu_HSV_1, gpu_fieldthresh0, gpu_fieldthresh1;

    int toggle, frame_count;


#ifdef TEST_LIVE_VIDEO

    // Camera video pipeline
    std::string pipeline = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#else

    // MP4 file pipeline
    std::string pipeline = "filesrc location=/home/nvidia/workspace/EE555_testvid/pong_video.mp4 ! qtdemux name=demux ! h264parse ! omxh264dec ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#endif

    std::cout << "Using pipeline: " << pipeline << std::endl;
 
    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cout << "Connection failed" << std::endl;
        return -1;
    }

    // Initialize 
    toggle = 0;
    frame_count = 0;

    // Input Quadrilateral Image Plane Coordinates
    Point2f inputQuad[4];

    // Output Quadrilateral World Plane Coordinates
    Point2f outputQuad[4];

    // Lamda Matrix
    cv::Mat lambda( 2, 4, CV_32FC1 );

    // The 4 points select the input quadrilateral. 
    // Begins from top-left, in clockwise order.
    // They are the sides of the box used as input.
    //
    // OpenCV coordinate system is based on rows and then columns.
    inputQuad[0] = Point2f(520,80);
    inputQuad[1] = Point2f(880,77);
    inputQuad[2] = Point2f(923,672);
    inputQuad[3] = Point2f(472,655);

    // 4 Points for mapping output to, from top-left clockwise order.
    outputQuad[0] = Point2f(437,0);
    outputQuad[1] = Point2f(842,0);
    outputQuad[2] = Point2f(842,719);
    outputQuad[3] = Point2f(437,719);

    // Get the Perspective Transformation Matrix, i.e. 'lambda'
    lambda = cv::getPerspectiveTransform(inputQuad,outputQuad);

    // Capture the first frame with GStreamer
    cap >> frame0;

    // Upload to GPU memory
    gpu_frame0.upload(frame0);

    // Use the HSV color filter
    cv::cuda::cvtColor(gpu_frame0,gpu_HSV_0,cv::COLOR_BGR2HSV);

    // Download from GPU memory
    gpu_HSV_0.download(HSV_0);

    // Field filter
    inRange(HSV_0, Scalar(HF_MIN,SF_MIN,VF_MIN), Scalar(HF_MAX,SF_MAX,VF_MAX), thresholdfilter_0);    

    morphOps(thresholdfilter_0);

    // Find Corners of the playing field    
    FindCorners(thresholdfilter_0, frame0);


    while (frame_count < 2500) {  //should be about 2970 total frames

        if (toggle == 0) {

            // Get a new frame from file
            cap >> frame1;

            // Upload to GPU memory
	        gpu_frame1.upload(frame1);


            // Use the HSV color filter
           cv::cuda::cvtColor(gpu_frame1,gpu_HSV_1,cv::COLOR_BGR2HSV);

            // Download from GPU memory
            gpu_HSV_1.download(HSV_1);

           // Field filter
            inRange(HSV_1, Scalar(HF_MIN,SF_MIN,VF_MIN), Scalar(HF_MAX,SF_MAX,VF_MAX), thresholdfilter_1);    

            morphOps(thresholdfilter_1);

            // Find Corners of the playing field    
            FindCorners(thresholdfilter_1, frame1);


            // Create Warp Perspective image, Frame 1
//            cv::cuda::warpPerspective(gpu_frame1, gpu_frame1_warped, gpu_lambda, gpu_frame1.size());
//            cv::cuda::warpPerspective(gpu_frame1,gpu_frame1_warped,lambda,gpu_frame1.size());
    
//            gpu_frame1_warped.download(frame1_warped);


	        // Convert the frames to grayscale (monochrome)
//            cv::cuda::cvtColor(gpu_frame1_warped,gpu_grayImage1,cv::COLOR_BGR2GRAY);
            toggle = 1;


        } 
        else {

	       // Get a new frame from file
            cap >> frame0;


            // Upload to GPU memory
	        gpu_frame0.upload(frame0);


            // Use the HSV color filter
           cv::cuda::cvtColor(gpu_frame0,gpu_HSV_0,cv::COLOR_BGR2HSV);

            // Download from GPU memory
           gpu_HSV_0.download(HSV_0);

            // Field filter
            inRange(HSV_0,Scalar(HF_MIN,SF_MIN,VF_MIN),Scalar(HF_MAX,SF_MAX,VF_MAX), thresholdfilter_0);    

 
            morphOps(thresholdfilter_0);
    

            // Find Corners of the playing field    
            FindCorners(thresholdfilter_0, frame0);

 
            // Create Warp Perspective image, Frame 0
//            cv::cuda::warpPerspective(gpu_frame0, gpu_frame0_warped, gpu_lambda, gpu_frame0.size());
//            cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());

//            gpu_frame0_warped.download(frame0_warped);



	        // Convert the frames to grayscale (monochrome)
//            cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);
            toggle = 0;


	    }
 
        // Compute the absolute value of the difference
//        cv::cuda::absdiff(gpu_grayImage0, gpu_grayImage1, gpu_differenceImage);


    	// Threshold the difference image
//        cv::cuda::threshold(gpu_differenceImage, gpu_thresholdImage, 50, 255,  cv::THRESH_BINARY);

//        gpu_thresholdImage.download(result);

    	// Find the location of any moving object and show the final frame
  	if (toggle == 0) {
            // Display Warped Image
//    	    imshow("Frame", frame0_warped);
//            imshow("ThreasholdColorFilter", thresholdfilter_0);


        /*    // Original Image with Quadrilateral
            cv::line(frame0,inputQuad[0],inputQuad[1],Scalar(0,255,0),2);
            cv::line(frame0,inputQuad[1],inputQuad[2],Scalar(0,255,0),2);
            cv::line(frame0,inputQuad[2],inputQuad[3],Scalar(0,255,0),2);
            cv::line(frame0,inputQuad[3],inputQuad[0],Scalar(0,255,0),2);
    	*/    imshow("Original", frame0);


         //   imshow("Threshold",thresholdfilter_0);    
    	}
    	else {

            // Display Warped Image
//    	    imshow("Frame", frame1_warped);
//            imshow("ThreasholdColorFilter", thresholdfilter_1);

        /*    // Original Image with Quadrilateral
            cv::line(frame1,inputQuad[0],inputQuad[1],Scalar(0,255,0),2);
            cv::line(frame1,inputQuad[1],inputQuad[2],Scalar(0,255,0),2);
            cv::line(frame1,inputQuad[2],inputQuad[3],Scalar(0,255,0),2);
            cv::line(frame1,inputQuad[3],inputQuad[0],Scalar(0,255,0),2);
    	*/    imshow("Original", frame1);
         //     imshow("Threshold",thresholdfilter_1);

        }

	    frame_count++;
//        std::cout << "Frame count: " << frame_count << '\n';
        cv::waitKey(1); //needed to show frame
    }
}

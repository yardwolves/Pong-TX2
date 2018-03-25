//-----------------------------------------------------------------------------------------------------------------
// File:   Lab7.cpp
// Author: Peter J. Tucker 
//
// Modified from Source File:   Lab_2.cpp
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

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

#define _CRT_SECURE_NO_WARNINGS

// Select Video Source
// The MP4 demo uses a ROI for better tracking of the moving object
#define TEST_LIVE_VIDEO

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

// Paddle Travel Speeds
#define LOW_MAX 6
#define MED_MAX 8
#define HIGH_MAX 11
#define FAST_MAX 15

using namespace cv;
using namespace std;

// Min and Max HSV filter values.
//Entire Field
int HF_MIN = 0;
int HF_MAX = 255;
int SF_MIN = 0;
int SF_MAX = 255;
int VF_MIN = 86; //84 (new is 86)
int VF_MAX = 255;
// Top Paddle 
int HT_MIN = 78;
int HT_MAX = 114;
int ST_MIN = 115;
int ST_MAX = 256;
int VT_MIN = 205;
int VT_MAX = 256;
// Bottom Paddle
int HB_MIN = 20;
int HB_MAX = 33;
int SB_MIN = 19;
int SB_MAX = 83;
int VB_MIN = 251;
int VB_MAX = 256;

// Time frame period in ms
double Ts = 33.3333;

// Inverse of n, for computing velocity
double invn = 1/(static_cast<double>(n));

// LSF Constructor
cv::Rect roi(XROI, YROI, LEN, HI);
Lsf *lsf =  new Lsf(roi, n);

// Corners for warp transform
vector<Point2f> corners;
int Corner_Cnt = 0;

// The pong ball
int theObject[2] = {0,0};
//bounding rectangle of the object, we will use the center of this as its position.
cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);

double destination = 0.0;
double paddle_location;

int fd;                           // File descriptor for the port
struct termios options;           // Terminal options

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

	//the element chosen here is a 5px by 5px rectangle
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

//    std::cout << "New frame" << std::endl;  

    // Allocating size of image    
    cv::Mat dst(src.size(), CV_32FC1, Scalar::all(0));
    cv::Mat dst_norm(src.size(), CV_32FC1, Scalar::all(0));

    int thresh = 210;  //180
    int apertureSize = aperture;
    int blockSize = blksz;
    double harrisK = 0.058;

    // Detecting corners
    cv::cornerHarris( src, dst, blockSize, apertureSize, harrisK, BORDER_DEFAULT );

    // Normalize
    cv::normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

    Corner_Cnt = 0;
    // Drawing circles around corners
    for( int j=0; j<dst_norm.rows; j++)
    {
        for( int i=0; i< dst_norm.cols; i++)
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                corners.push_back(Point2f(i,j));
                ++Corner_Cnt;

                cv::circle(cameraFeed, Point(i,j), 5, Scalar(55,255,55), 2, 8, 0);
                std::cout << "Corner Points: (" << i << "," << j << ")" << '\n';

            }
        }
    }
}


//-----------------------------------------------------------------------------------------------------------------
// Search for Paddles
//-----------------------------------------------------------------------------------------------------------------
void FindPaddle(cv::Mat warped, cv::Mat &cameraFeed, cv::Rect &roi) {

    // Two vectors for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::Mat temp;
    cv::Rect Paddle = cv::Rect(0, 0, 0, 0);

    // Copy HSV image to temp
    warped.copyTo(temp);

    cv::Mat roi_temp = temp(roi);

    // Creates a rectangle around the Region of Interest
//    cv::rectangle(cameraFeed,Point(roi.x,roi.y),Point(roi.x+roi.width,roi.y+roi.height),Scalar(255,55,255),2);

    cv::findContours(roi_temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0) 
    {
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));
        Paddle = boundingRect(largestContourVec.at(0));
    }

    Paddle.x = Paddle.x+roi.x;
    Paddle.y = Paddle.y+roi.y;

    // Draws the Paddle
    if(contours.size()>0) 
    {
        cv::rectangle(cameraFeed, Point(Paddle.x, Paddle.y), Point(Paddle.x+Paddle.width, Paddle.y+Paddle.height), Scalar(255,55,55),2);
        paddle_location = (Paddle.x)/2;
    }
    else    
        paddle_location = 0;

    // Write object's position to screen
//    putText(cameraFeed,"(" + intToString(Paddle.x)+","+intToString(Paddle.y)+")",Point(Paddle.x,Paddle.y),1,1,Scalar(255,0,0),2);  
}


//-----------------------------------------------------------------------------------------------------------------
// Search for Moving Object
//-----------------------------------------------------------------------------------------------------------------
void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed){

    // The '&' operator for cameraFeed will allow us to take the values passed
    // into the function and manipulate them, rather than just working with a copy.
    // eg. we draw to the cameraFeed to be displayed in the main() function.

    bool objectDetected = false;
    int xpos, ypos;

    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Mat temp;

    // Copies thresholdImage (frame from main) to temp
    thresholdImage.copyTo(temp);

//#ifdef TEST_LIVE_VIDEO

    //find contours of filtered image using openCV findContours function
//    cv::findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

//#else

    // Creates a rectangle around the Region of Interest
    cv::rectangle(cameraFeed,Point(XROI,YROI),Point(XROI+LEN,YROI+HI),Scalar(255,55,55),2);

    // Creates a Region of Interest(ROI) to find the moving ball
    cv::Rect roi(XROI, YROI, LEN, HI);

    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
    // retrieves external contours

//#endif

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)
	objectDetected = true;
    else 
	objectDetected = false;
 
    if(objectDetected){

        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));

        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));

        xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
 
        //update the objects positions by changing the 'theObject' array values
        theObject[0] = xpos , theObject[1] = ypos;
    }

    //make some temp x and y variables so we dont have to type out so much
    int x = theObject[0]+XROI;
    int y = theObject[1]+YROI;
     
    //write the position of the object to the screen
    putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
 
    // Print the object position
    //cout << xpos << " " << ypos << endl;
}


//-----------------------------------------------------------------------------------------------------------------
// Moving Paddle to Hit Ball
//-----------------------------------------------------------------------------------------------------------------
void gitThatBall() {

    char write_buffer[1];
    char read_buffer[1];
    int  bytes_written;  
    int  bytes_read; 
    struct termios options;           // Terminal options
    double difference = 0;
    bool serial_avail = 0;

    

    // Check response
    if(read(fd, &read_buffer, 1) != 1)
    {
        //std::cout << "Acknoledge Write Command " << read_buffer[0] << std::endl;
        serial_avail = 1;
    }
    else{
        // Where the paddle is related to where it needs to be
        int difference = destination - paddle_location;
        
        char buffer[10];
    
        memset(&write_buffer[0], 0, sizeof(write_buffer)); // Clear the array
        if( difference > 0 )
            strcpy(write_buffer, "R");      // Go right command
        else
            strcpy(write_buffer, "L");      // Go left command

        difference = abs(difference);
    
        if( difference < 10)
            sprintf(buffer, "%d", LOW_MAX);

        else if( difference < 50)
            sprintf(buffer, "%d", MED_MAX);

        else if( difference < 100)
            sprintf(buffer, "%d", HIGH_MAX);

        else
            sprintf(buffer, "%d", FAST_MAX);        
    
        strcat(write_buffer, buffer);
        bytes_written = write(fd, &write_buffer, strlen(write_buffer));

        serial_avail = 0;
    }
}


//-----------------------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------------------
int main() {

    // OpenCV frame matrices
    cv::Mat frame0, frame1, frame0_warped, frame1_warped, result, HSV_0, HSV_1, thresholdfilter_0, thresholdfilter_1, gray0, gray1, topfilter0, topfilter1, bottomfilter0, bottomfilter1, threshold;

    cv::cuda::GpuMat gpu_frame0, gpu_frame1, gpu_frame0_warped, gpu_frame1_warped, gpu_grayImage0, gpu_grayImage1, gpu_differenceImage, gpu_thresholdImage, gpu_HSV_0, gpu_HSV_1, gpu_fieldthresh0, gpu_fieldthresh1;

    int toggle, frame_count;

    struct termios options;           // Terminal options
    int fd;                           // File descriptor for the port

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

    fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);   // Open tty device for RD and WR

    if(fd == 1) {
        printf("\n  Error! in Opening ttyUSB0\n");
    }
    else
        printf("\n  ttyUSB0 Opened Successfully\n");

    tcgetattr(fd, &options);               // Get the current options for the port
    cfsetispeed(&options, B115200);        // Set the baud rates to 115200          
    cfsetospeed(&options, B115200);                   
    options.c_cflag |= (CLOCAL | CREAD);   // Enable the receiver and set local mode           
    options.c_cflag &= ~PARENB;            // No parity                 
    options.c_cflag &= ~CSTOPB;            // 1 stop bit                  
    options.c_cflag &= ~CSIZE;             // Mask data size         
    options.c_cflag |= CS8;                // 8 bits
    options.c_cflag &= ~CRTSCTS;           // Disable hardware flow control    

// Enable data to be processed as raw input
    options.c_lflag &= ~(ICANON | ECHO | ISIG);
     
    tcsetattr(fd, TCSANOW, &options);      // Apply options immediately
    fcntl(fd, F_SETFL, FNDELAY);    
    


    // Initialize 
    toggle = 0;
    frame_count = 0;

    // Creates a Region of Interest(ROI) to find the upper paddle
    cv::Rect pad_T(XROIT, YROIT, LENT, HIT);

    // Input Quadrilateral Image Plane Coordinates
    Point2f inputQuad[4];

    // Output Quadrilateral World Plane Coordinates
    Point2f outputQuad[4];

    // Lamda Matrix
    cv::Mat lambda( 2, 4, CV_32FC1 );

    // Finds first set of 4 corners for lambda Warp Perspective 
    do
    {
        // Clear corner vector Points
        corners.erase(corners.begin(), corners.begin()+Corner_Cnt);
    
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
    
        // Increases threshold block, eliminates small noise 
        morphOps(thresholdfilter_0);

        // Find Corners of the playing field    
        FindCorners(thresholdfilter_0, frame0);

    } while( Corner_Cnt != 4); 

    std::cout << "Points: (" <<  corners[0].x << "," << corners[0].y <<
            "), (" << corners[1].x << "," << corners[1].y <<
            "), (" << corners[2].x << "," << corners[2].y <<
            "), (" << corners[3].x << "," << corners[3].y << ") " << std::endl;

   
    // The 4 points select the input quadrilateral. 
    // Begins from top-left, in clockwise order.
    // They are the sides of the box used as input.
    // 
    // OpenCV coordinate system is based on rows and then columns.
    // FindCorners starts at top left and moves counter clockwise.
    inputQuad[3] = corners[1];
    inputQuad[0] = corners[0];
    inputQuad[1] = corners[3];
    inputQuad[2] = corners[2];

//    inputQuad[0] = Point2f(520,80);
//    inputQuad[1] = Point2f(880,77);
//    inputQuad[2] = Point2f(923,672);
//    inputQuad[3] = Point2f(472,655);

    // 4 Points for mapping output to, from top-left clockwise order.
    outputQuad[0] = Point2f(437,0);
    outputQuad[1] = Point2f(842,0);
    outputQuad[2] = Point2f(842,719);
    outputQuad[3] = Point2f(437,719);

    // Get the Perspective Transformation Matrix, i.e. 'lambda'
    lambda = cv::getPerspectiveTransform(inputQuad,outputQuad);

    // Create Warp Perspective image, Frame 0
    cv::cuda::warpPerspective(gpu_frame0, gpu_frame0_warped, lambda, gpu_frame0.size());

    // Convert the frames to grayscale (monochrome)
    cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);

    while (frame_count < 2500) {  //should be about 2970 total frames

        if (toggle == 0) {

            // Get a new frame from file
            cap >> frame1;

            // Upload to GPU memory
	        gpu_frame1.upload(frame1);

            // Create Warp Perspective image, Frame 1
            cv::cuda::warpPerspective(gpu_frame1,gpu_frame1_warped,lambda,gpu_frame1.size());

            // Use the HSV color filter
            cv::cuda::cvtColor(gpu_frame1_warped,gpu_HSV_1,cv::COLOR_BGR2HSV);

            // Download HSV from GPU memory
            gpu_HSV_1.download(HSV_1);

            // Download warped image from GPU
            gpu_frame1_warped.download(frame1_warped);

            // Threshold range for Top Paddle
            inRange(HSV_1, Scalar(HT_MIN,ST_MIN,VT_MIN), Scalar(HT_MAX,ST_MAX,VT_MAX), topfilter1);

            // Find Paddle
            FindPaddle(topfilter1, frame1_warped, pad_T);

	        // Convert the frames to grayscale (monochrome)
            cv::cuda::cvtColor(gpu_frame1_warped,gpu_grayImage1,cv::COLOR_BGR2GRAY);
            toggle = 1;

        } 
        else {

	        // Get a new frame from file
            cap >> frame0;

            // Upload to GPU memory
	        gpu_frame0.upload(frame0);

            // Create Warp Perspective image, Frame 1
            cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());

            // Use the HSV color filter
            cv::cuda::cvtColor(gpu_frame0_warped,gpu_HSV_0,cv::COLOR_BGR2HSV);

            // Download from GPU memory
            gpu_HSV_0.download(HSV_0);

            // Download warped image from GPU
            gpu_frame0_warped.download(frame0_warped);

            // Threshold range for Top Paddle
            inRange(HSV_0, Scalar(HT_MIN,ST_MIN,VT_MIN), Scalar(HT_MAX,ST_MAX,VT_MAX), topfilter0);

            // Find Paddle
            FindPaddle(topfilter0, frame0_warped, pad_T);

	        // Convert the frames to grayscale (monochrome)
            cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);

            toggle = 0;
	    }
 
        // Compute the absolute value of the difference
        cv::cuda::absdiff(gpu_grayImage0, gpu_grayImage1, gpu_differenceImage);

    	// Threshold the difference image
        cv::cuda::threshold(gpu_differenceImage, gpu_thresholdImage, 50, 255,  cv::THRESH_BINARY);

        gpu_thresholdImage.download(result);

    	// Find the location of any moving object and show the final frame
  	    if (toggle == 0) {
            //searchForMovement(thresholdImage,frame);
            searchForMovement(result,frame0_warped);

            // Least squares function
            lsf->addPoint(theObject,frame0_warped,destination);
            
            // Move Paddle
            gitThatBall();

            // Display Warped Image
    	    imshow("Frame", frame0_warped);

//    	    imshow("Original", frame0);

    	}
    	else {
            //searchForMovement(thresholdImage,frame1);
            searchForMovement(result,frame1_warped);

            // Least squares function
            lsf->addPoint(theObject,frame1_warped,destination);
       
            // Move Paddle
            gitThatBall();

            // Display Warped Image
    	    imshow("Frame", frame1_warped);

//    	    imshow("Original", frame1);
        }

	    frame_count++;
//        std::cout << "Frame count: " << frame_count << '\n';
        cv::waitKey(1); //needed to show frame
    }
    // Close connection port
    close(fd);
}

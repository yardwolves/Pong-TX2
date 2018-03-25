//-----------------------------------------------------------------------------------------------------------------
// File:    color_filter.cpp 
//          Color Filter
// Author:  Peter J. Tucker 
//          OIT-PM, Winter 2018
//          Embedded Systems 2, EE 555
//
//          Modified from objectTrackingTutorial.cpp by Kyle Hounslow.
//
//-----------------------------------------------------------------------------------------------------------------

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>
#include <vector>
#include <opencv2/cudacodec.hpp>
#include "color_filter.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace cv;
using namespace std;

ColorFilter::ColorFilter(bool useMorphOps) {

    m_useMorphOps = useMorphOps;
    HUE_MIN = 0;
    HUE_MAX = 256;
    SAT_MIN = 0;
    SAT_MAX = 256;
    VALUE_MIN = 0;
    VALUE_MAX = 256;
    TrackbarName[50];

    //names that will appear at the top of each window
    string windowName = "Original Image";
    string windowName1 = "HSV Image";
    string windowName2 = "Thresholded Image";
    string windowName3 = "After Morphological Operations";
    string trackbarWindowName = "Trackbars";

    // Copies Mat camerFeed to each Mat for allocating memory
    HSV;
    threshold;
    erodeElement;
    dilateElement;
    createTrackbars();
}

/*
void ColorFilter::on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}*/

string ColorFilter::intToString(int number){

	std::stringstream ss;
	ss << number;
	return ss.str();
}



void ColorFilter::createTrackbars(){

	//create window for trackbars
    namedWindow(trackbarWindowName,0);

	//create memory to store trackbar name on window

	sprintf( TrackbarName, "HUE_MIN", HUE_MIN);
	sprintf( TrackbarName, "HUE_MAX", HUE_MAX);
	sprintf( TrackbarName, "SAT_MIN", SAT_MIN);
	sprintf( TrackbarName, "SAT_MAX", SAT_MAX);
	sprintf( TrackbarName, "VALUE_MIN", VALUE_MIN);
	sprintf( TrackbarName, "VALUE_MAX", VALUE_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    cv::createTrackbar("HUE_MIN", trackbarWindowName, &HUE_MIN, HUE_MAX, 0,0);
    cv::createTrackbar("HUE_MAX", trackbarWindowName, &HUE_MAX, HUE_MAX, 0,0);
    cv::createTrackbar("SAT_MIN", trackbarWindowName, &SAT_MIN, SAT_MAX, 0,0);
    cv::createTrackbar("SAT_MAX", trackbarWindowName, &SAT_MAX, SAT_MAX, 0,0);
    cv::createTrackbar("VALUE_MIN", trackbarWindowName, &VALUE_MIN, VALUE_MAX,0, 0);
    cv::createTrackbar("VALUE_MAX", trackbarWindowName, &VALUE_MAX, HUE_MAX, 0, 0);

}


//create structuring element that will be used to "dilate" and "erode" image.
void ColorFilter::morphOps(Mat &thresh){

	//the element chosen here is a 3px by 3px rectangle
	erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));

    //dilate with larger element so make sure object is nicely visible
	dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}


void ColorFilter::videomain(cv::Mat &cameraFeed)
{

	//create slider bars for HSV filtering
	//createTrackbars();


	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
//	while(1){
		//store image to matrix
//		capture >> cameraFeed;
		//convert frame from BGR to HSV colorspace
		cv::cuda::cvtColor(cameraFeed,HSV,cv::COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix

HSV.download(frame0_warped);

		inRange(HSV,Scalar(HUE_MIN,SAT_MIN,VALUE_MIN),Scalar(HUE_MAX,SAT_MAX,VALUE_MAX),threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		//if(m_useMorphOps)
		morphOps(threshold);

//Not using that here
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
//		if(trackObjects)
//			trackFilteredObject(x,y,threshold,cameraFeed);

		//show frames 
		imshow(windowName2,threshold);
//		imshow(windowName,cameraFeed);
//		imshow(windowName1,HSV);
		

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
	waitKey(1);
//	}
//	return 0;
}


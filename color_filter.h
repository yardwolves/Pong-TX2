//-----------------------------------------------------------------------------------------------------------------
// File:    color_filter.h 
//          Color Filter Header
// Author:  Peter J. Tucker 
//          OIT-PM, Winter 2018
//          Embedded Systems 2, EE 555
//
//-----------------------------------------------------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <vector>
#include <stdio.h>
#include <opencv2/cudacodec.hpp>

using namespace cv;
using namespace std;

#ifndef color_filter_h
#define color_filter_h

class ColorFilter {
    private:

     
        //names that will appear at the top of each window
const string windowName;
    const string windowName1;
    const string windowName2;
    const string windowName3;
    const string trackbarWindowName;
        bool m_useMorphOps;

    public:
        // class constructor
        ColorFilter(bool useMorphOps);
        int HUE_MIN;
        int HUE_MAX;
        int SAT_MIN;
        int SAT_MAX;
        int VALUE_MIN;
        int VALUE_MAX;

        cv::Mat erodeElement;
	    cv::Mat dilateElement;
	    cv::Mat HSV;
	    cv::Mat threshold;
        char TrackbarName[50];

        string intToString(int number);
        void on_trackbar( int, void* );
        void createTrackbars();
        void morphOps(Mat &thresh);
        void videomain(cv::Mat &cameraFeed);


};
#endif


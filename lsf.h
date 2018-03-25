//-----------------------------------------------------------------------------------------------------------------
// File:    lsf.h 
//          Least Squares Fit Header
// Author:  Peter J. Tucker 
//          OIT-PM, Winter 2018
//          Embedded Systems 2, EE 555
//
//-----------------------------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <utility>
#include <vector>

using namespace cv;
using namespace std;

#ifndef lsf_h
#define lsf_h

class Lsf {
    private:
        double m_x_slope;
        double m_y_slope;

        unsigned int m_xmax;
        unsigned int m_xmin;
        unsigned int m_ymax;
        unsigned int m_ymin;

        double m_paramc;
        double m_reps;
        double m_aeps;
        cv:: Rect m_ROI;
        unsigned int m_N;
        int m_n;
        
    public:
        Lsf(cv::Rect roi, unsigned int N);
        unsigned int m_Cnt;        
        void addPoint(int *ObjectPos, cv::Mat &cameraFeed, double &destination);
        unsigned int m_x;
        unsigned int m_y;
        std::vector<Point> coordpts;
        std::vector<Point> prevpts;
        cv::Vec4f m_line;
        double m_X0;
        double m_Y0;
        double m_X1;
        double m_Y1;
        int m_xpos;
        int m_ypos;
        int m_Yfin;
        int m_Xfin;
        int m_xp;
        int m_xn;
        int m_dirx;
        //int diry;
        void directions();
        bool m_score_toggle;
        bool m_Ydir_toggle;
        int m_found;
        int m_a;
        void findPath( /*cv::Mat &cameraFeed,*/ std::vector<Point> &plots );
        int s1;
        int s2; 
        int s3;
        int i;
        int k;
        std::vector<Point> plots;

};
#endif

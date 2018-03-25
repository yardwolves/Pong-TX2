//-----------------------------------------------------------------------------------------------------------------
// File:    lsf.cpp 
//          Least Squares Fit
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
#include <stdio.h>
#include <math.h>
#include <vector>
#include <utility>
#include "lsf.h"

using namespace cv;
using namespace std;


Lsf::Lsf(cv::Rect roi, unsigned int N) {
    m_N = N;
    m_ROI = roi;
    m_Cnt = 0;
    m_paramc = 0.0;
    m_reps = 0.01;
    m_aeps = 0.01;
    m_a = 1;
    m_xp = 0;
    m_xn = 0;
    m_dirx = 0;
    m_found = 0;
    m_Ydir_toggle = 0;
    m_xpos = 0;
    m_ypos = 0;
    m_Yfin = 0;
    m_Xfin = 0;
    vector<Point> coordpts(m_N);
    vector<Point> prevpts(m_N);
    vector<Point> plots(256);
    m_xmax = m_ROI.x + m_ROI.width;
    m_xmin = m_ROI.x;
    m_ymax = m_ROI.y + m_ROI.height;
    m_ymin = m_ROI.y;
    Vec4f m_line;
    m_score_toggle = 0;
    i = 0;
    k = 0;
}


// Comparing only half the vector values
void Lsf::directions(){
    m_xp = 0;
    m_xn = 0;
            
    for ( i = 0; i < (m_N); ++i)
    {
        if (coordpts[i].x > prevpts[i].x)
            m_xp++;
        else if (coordpts[i].x < prevpts[i].x)
            m_xn++;
    }
    
    // Difference of pos and neg is less 6 (not monotonic)
    if( m_xp > 3 && m_xn > 3 )
        m_dirx = 0;
    // Negative direction
    else if( m_xn > 2 )
        m_dirx = -1;
    // Positive direction (or no x change)
    else  
        m_dirx = 1;
}


void Lsf::findPath(/*cv::Mat &cameraFeed,*/ std::vector<Point> &plots) {
    
    m_Y1 = (m_a * m_y_slope * (m_X1 - m_X0)) + m_Y0;
//std::cout << "X0: " << m_X0 << ", Y0: " << m_Y0 << ", X1:" << m_X1 << ", Y1: " << m_Y1 << '\n';
 
    if( m_Y1 > (m_ymax-2) || m_Y1 < (m_ymin+2) )
    {
        m_found = 1;
        m_Xfin = (m_a * m_x_slope * (m_Yfin - m_Y0)) + m_X0;
        // Using Xfin and Yfin to not corrupt X1 and Y1 from loop iteration
        // Yfin is loaded from direction towards Ymin or Ymax 
//        cv::line(cameraFeed,Point(m_xpos,m_ypos),Point(m_Xfin,m_Yfin),Scalar(s1,s2,s3),2);   
        plots.push_back(Point(m_xpos,m_ypos));
        plots.push_back(Point(m_Xfin,m_Yfin));
    }

    else 
    {
//        cv::line(cameraFeed,Point(m_xpos,m_ypos),Point(m_X1,m_Y1),Scalar(s1,s2,s3),2);
        //std::cout << "Downward In loop" << '\n';
              //        m_found = 0;
        plots.push_back(Point(m_xpos,m_ypos));
        plots.push_back(Point(m_X1,m_Y1));

        m_X0 = m_X1;
        m_Y0 = m_Y1;

        m_a = (-1)*m_a;
        
        if( m_X1 == m_xmax )
            m_X1 = m_xmin;
        else
            m_X1 = m_xmax;
        
        m_xpos = m_X0;
        m_ypos = m_Y0;
     
    }

}


void Lsf::addPoint(int *ObjectPos, cv::Mat &cameraFeed, double &destination) {

    // Get the current x and y postions
    m_x = ObjectPos[0] + m_ROI.x;
    m_y = ObjectPos[1] + m_ROI.y;

    // Load parametric vector points
    // Checks to see that new ball is on field
    if ( (m_Cnt < m_N) && (m_y > (m_ymin+20)) && (m_y < (m_ymax-20)) )    
    {
        coordpts.push_back(Point(m_x,m_y));
        ++m_Cnt;
        m_score_toggle = 0;
        return;
    }

    // If there has been a goal
    else if ( m_score_toggle == 1)
        return;
    
    
    // Load previous vector coordinates
    prevpts = coordpts;
    
    // Erase oldest value, Load current with new position
    coordpts.erase(coordpts.begin());
    coordpts.push_back(Point(m_x,m_y));


    directions();

    cv::fitLine(coordpts, m_line, DIST_L2, 0, 0.001, 0.001);
    

    // Adding a slight to make sure slope is not infinite    
    if( m_line[0] == 0 )
        m_y_slope = (m_line[1])/(0.0001);
    else if( m_line[1] == 0)
    {
        m_y_slope = (0.0001)/(m_line[0]);
        m_x_slope = (m_line[0])/(0.0001);
    }
    else
    {
        m_y_slope = (m_line[1])/(m_line[0]);
        m_x_slope = (m_line[0])/(m_line[1]);
    }


    m_X0 = m_line[2];
    m_Y0 = m_line[3];
      
    m_xpos = m_x;
    m_ypos = m_y;

    
//    std::cout << "value of dirx: " << m_dirx << ", vy: " << m_line[1] << '\n';

    // Don't plot non-monotonic line
    if( m_dirx == 0 )
    {
        return;
    }
    // Moving Upward & Left (Right & Up when screen upright)
    else if( m_line[1] > 0 && m_dirx < 0 )
    {
        m_Ydir_toggle = 0;
        m_X1 = m_xmin;
        m_Yfin = m_ymin;
        s1 = 255; s2 = 55; s3 = 55;
//        std::cout << "Upward & Left" << '\n';
    }
    // Moving Downward & Left (Left & Up when screen upright)
    else if( m_line[1] < 0 && m_dirx < 0 )
    {
        m_Ydir_toggle = 1;
        m_X1 = m_xmin;
        m_Yfin = m_ymax;
        s1 = 55; s2 = 55; s3 = 255;
//        std::cout << "Downward & Left" << '\n';
    }
    // Moving Upward & Right (Right & Down when screen upright)
    else if( m_line[1] < 0)
    {
        m_Ydir_toggle = 0;
        m_X1 = m_xmax;
        m_Yfin = m_ymin;
        s1 = 255; s2 = 55; s3 = 55;
//        std::cout << "Upward & Right" << '\n';
    }
    // Ball moving Downward & Right (Left & Down when screen upright)
    else if( m_line[1] > 0 )
    {
        m_Ydir_toggle = 1;
        m_X1 = m_xmax;
        m_Yfin = m_ymax;
        s1 = 55; s2 = 55; s3 = 255;
//        std::cout << "Downward & Right" << '\n';
    }
//    std::cout << "Ydir toggle before loop: " << m_Ydir_toggle << '\n';    

    m_a = 1;
    m_found = 0;
    k = 0;
    do
    {
        findPath(/*cameraFeed,*/ plots);
        k++;
//        std::cout << "Dowhile iterations: " << i << '\n';
    }
    while( m_found == 0 );

    destination = plots[k].x;

    for( k=0; k<plots.size(); k+=2 )
    {
        cv::line(cameraFeed, plots[k], plots[k+1],Scalar(s1,s2,s3),2);
    }
    // Clear the vector plot point array
    plots.erase( plots.begin(), plots.begin()+plots.size() );


    // Clear vectors if score has been made (vx and vy = 0)
    // And clear fill counter
    if ( m_line[1] == 0 )
    {
        m_Cnt = 0;
        coordpts.erase(coordpts.begin(), coordpts.begin()+m_N); 
        // Increment score here in future
        m_score_toggle = 1;  
    }

};



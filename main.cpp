#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <raspicam/raspicam_cv.h>

#define FRAME_RATE 10

const int max_value_H = 360/2;
const int max_value = 255;
int low_HRed = 0, low_SRed = 0, low_VRed = 0;
int high_HRed = max_value_H, high_SRed = max_value, high_VRed = max_value;
int low_HYel = 0, low_SYel = 0, low_VYel = 0;
int high_HYel = max_value_H, high_SYel = max_value, high_VYel = max_value;
int diffLeft = 50;
int diffRight = 50;

//Red Trackbar
static void on_low_H_Red_trackbar(int, void *)
{
    low_HRed = cv::min(high_HRed-1, low_HRed);
    cv::setTrackbarPos("Low H Red", "Config", low_HRed);
}
static void on_high_H_Red_trackbar(int, void *)
{
    high_HRed = cv::max(high_HRed, low_HRed+1);
    cv::setTrackbarPos("High H Red", "Config", high_HRed);
}
static void on_low_S_Red_trackbar(int, void *)
{
    low_SRed = cv::min(high_SRed-1, low_SRed);
    cv::setTrackbarPos("Low S Red", "Config", low_SRed);
}
static void on_high_S_Red_trackbar(int, void *)
{
    high_SRed = cv::max(high_SRed, low_SRed+1);
    cv::setTrackbarPos("High S Red", "Config", high_SRed);
}
static void on_low_V_Red_trackbar(int, void *)
{
    low_VRed = cv::min(high_VRed-1, low_VRed);
    cv::setTrackbarPos("Low V Red", "Config", low_VRed);
}
static void on_high_V_Red_trackbar(int, void *)
{
    high_VRed = cv::max(high_VRed, low_VRed+1);
    cv::setTrackbarPos("High V Red", "Config", high_VRed);
}
//Yellow Trackbar
static void on_low_H_Yel_trackbar(int, void *)
{
    low_HYel = cv::min(high_HYel-1, low_HYel);
    cv::setTrackbarPos("Low H Yel", "Config", low_HYel);
}
static void on_high_H_Yel_trackbar(int, void *)
{
    high_HYel = cv::max(high_HYel, low_HYel+1);
    cv::setTrackbarPos("High H Yel", "Config", high_HYel);
}
static void on_low_S_Yel_trackbar(int, void *)
{
    low_SYel = cv::min(high_SYel-1, low_SYel);
    cv::setTrackbarPos("Low S Yel", "Config", low_SYel);
}
static void on_high_S_Yel_trackbar(int, void *)
{
    high_SYel = cv::max(high_SYel, low_SYel+1);
    cv::setTrackbarPos("High S Yel", "Config", high_SYel);
}
static void on_low_V_Yel_trackbar(int, void *)
{
    low_VYel = cv::min(high_VYel-1, low_VYel);
    cv::setTrackbarPos("Low V Yel", "Config", low_VYel);
}
static void on_high_V_Yel_trackbar(int, void *)
{
    high_VYel = cv::max(high_VYel, low_VYel+1);
    cv::setTrackbarPos("High V Yel", "Config", high_VYel);
}
//Diff params
static void on_diffLeft_trackbar(int, void *)
{
    cv::setTrackbarPos("Diff Left", "Config", diffLeft);
}
static void on_diffRight_trackbar(int, void *)
{
    cv::setTrackbarPos("Diff Right", "Config", diffRight);
}



int main(int argc, char *argv[])
{
    raspicam::RaspiCam_Cv cam;
    cam.set( cv::CAP_PROP_FORMAT, CV_8UC3 );
    if (!cam.open())
    {
        std::cerr << "Can't open raspi camera." << std::endl;
        return -1;
    }
    low_HRed = 170;
    high_HRed = 179;

    low_SRed = 150;
    high_SRed = 255;

    low_VRed = 60;
    high_VRed = 255;

    low_HYel = 20;
    high_HYel = 80;

    low_SYel = 100;
    high_SYel = 255;

    low_VYel = 100;
    high_VYel = 255;

    std::ifstream conf;
    conf.open("config.txt");
    if (conf.is_open())
    {
        conf >> low_HRed >> high_HRed >> low_SRed >> high_SRed >> low_VRed >> high_VRed;
        conf >> low_HYel >> high_HYel >> low_SYel >> high_SYel >> low_VYel >> high_VYel;
        conf >> diffLeft >> diffRight;
        conf.close();
    }
    
    cv::namedWindow("Config", cv::WINDOW_AUTOSIZE);
   // cv::resizeWindow("Config", 800, 720);
    

    cv::namedWindow("Video", cv::WINDOW_NORMAL);
    cv::resizeWindow("Video", 1280, 720);
    
        // Trackbars to set thresholds for HSV values
    cv::createTrackbar("Low H Red", "Config", nullptr, max_value_H, on_low_H_Red_trackbar);
    cv::createTrackbar("High H Red", "Config",nullptr, max_value_H, on_high_H_Red_trackbar);
    cv::createTrackbar("Low S Red", "Config", nullptr, max_value, on_low_S_Red_trackbar);
    cv::createTrackbar("High S Red", "Config", nullptr, max_value, on_high_S_Red_trackbar);
    cv::createTrackbar("Low V Red", "Config", nullptr, max_value, on_low_V_Red_trackbar);
    cv::createTrackbar("High V Red", "Config", nullptr,max_value, on_high_V_Red_trackbar);
    
    cv::createTrackbar("Low H Yel", "Config", nullptr, max_value_H, on_low_H_Yel_trackbar);
    cv::createTrackbar("High H Yel", "Config", nullptr, max_value_H, on_high_H_Yel_trackbar);
    cv::createTrackbar("Low S Yel", "Config", nullptr, max_value, on_low_S_Yel_trackbar);
    cv::createTrackbar("High S Yel", "Config", nullptr, max_value, on_high_S_Yel_trackbar);
    cv::createTrackbar("Low V Yel", "Config", nullptr, max_value, on_low_V_Yel_trackbar);
    cv::createTrackbar("High V Yel", "Config", nullptr, max_value, on_high_V_Yel_trackbar);
    cv::createTrackbar("diffLeft", "Config", nullptr, 300, on_diffLeft_trackbar);
    cv::createTrackbar("diffRight", "Config", nullptr, 300, on_diffRight_trackbar);

    cv::setTrackbarPos("Low H Red", "Config", low_HRed);    
    cv::setTrackbarPos("High H Red", "Config", low_HRed);    
    cv::setTrackbarPos("Low S Red", "Config", low_HRed);    
    cv::setTrackbarPos("High S Red", "Config", low_HRed);    
    cv::setTrackbarPos("Low V Red", "Config", low_HRed);    
    cv::setTrackbarPos("High V Red", "Config", low_HRed);    
    
    cv::setTrackbarPos("Low H Yel", "Config", low_HYel);    
    cv::setTrackbarPos("High H Yel", "Config", high_HYel);    
    cv::setTrackbarPos("Low S Yel", "Config", low_SYel);    
    cv::setTrackbarPos("High S Yel", "Config", high_SYel);    
    cv::setTrackbarPos("Low V Yel", "Config", low_VYel);    
    cv::setTrackbarPos("High V Yel", "Config", high_VYel);    
    
    cv::setTrackbarPos("diffLeft", "Config", diffLeft);    
    cv::setTrackbarPos("diffRight", "Config", diffRight);

    cv::Mat frame;
    int frameCounter = 0;
    for (;;)
    {
        cam.grab();
        cam.retrieve(frame);
        if (frame.empty())
        {
            break;
        }

        cv::Mat imgHSV;
        cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV);
        cv::Mat imgTreshRed;
        cv::inRange(imgHSV, cv::Scalar(low_HRed, low_SRed, low_VRed), cv::Scalar(high_HRed,
                    high_SRed, high_VRed), imgTreshRed);

        cv::Mat imgTreshYel;
        cv::inRange(imgHSV, cv::Scalar(low_HYel, low_SYel, low_VYel), cv::Scalar(high_HYel,
                    high_SYel, high_VYel), imgTreshYel);
        
        
        //RED
        cv::erode(imgTreshRed, imgTreshRed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate( imgTreshRed, imgTreshRed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

        //morphological closing (removes small holes from the foreground)
        cv::dilate( imgTreshRed, imgTreshRed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(imgTreshRed, imgTreshRed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

        //Yellow
        cv::erode(imgTreshYel, imgTreshYel, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate( imgTreshYel, imgTreshYel, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

        //morphological closing (removes small holes from the foreground)
        cv::dilate( imgTreshYel, imgTreshYel, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(imgTreshYel, imgTreshYel, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );




        std::vector<std::vector<cv::Point> > contoursRed;
        cv::Mat contourOutputRed = imgTreshRed.clone();
        cv::findContours( contourOutputRed, contoursRed, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );

        std::vector<std::vector<cv::Point> > contoursYel;
        cv::Mat contourOutputYel = imgTreshYel.clone();
        cv::findContours( contourOutputYel, contoursYel, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );

        int centerX = frame.size().width/2;
        int centerY = frame.size().height/2;

        cv::circle(frame, cv::Point(centerX, centerY), 4, cv::Scalar(0, 255, 0), cv::FILLED);

        double redDist = centerX;
        double redArea = 0;
        int redX = -1;
        int redY = -1;
        double yelDist = centerX;
        double yelArea = 0;
        int yelX = -1;
        int yelY = -1;
        double maxDist = std::sqrt(std::pow(centerX - frame.size().width, 2) + std::pow(centerY - frame.size().height/2, 2));

        for (size_t idx = 0; idx < contoursRed.size(); idx++) {
            cv::drawContours(frame, contoursRed, idx, cv::Scalar(0, 255, 0));
            cv::Moments m = cv::moments(contoursRed[idx]);
            int cX = (int) m.m10 / m.m00;
            int cY = (int) m.m01 / m.m00;

            double area = cv::contourArea(contoursRed[idx]);
            double dist = std::sqrt(std::pow(centerX - cX, 2) + std::pow(centerY - cY, 2));
            
            if (area > redArea)
            {
                redX = cX;
                redY = cY;
                redDist = dist;
            }
            redArea = std::max(area, redArea);
            
        }
        if (redX == -1 && redY == -1)
        {
            redX = 0;
            redY = frame.size().height/2;
        }

        cv::circle(frame, cv::Point(redX, redY), 7, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, "Left", cv::Point(redX, redY + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1.5);

       // std::cout << "dist: " << redDist << " max dist: " << maxDist  << "dv: " << static_cast<int>(5 * std::abs(redDist)/maxDist + 1) << std::endl;
        cv::arrowedLine(frame, cv::Point(redX, redY), cv::Point(centerX, centerY), cv::Scalar(0, 0, 255), static_cast<int>(5 * redDist/maxDist + 1));
         


        for (size_t idx = 0; idx < contoursYel.size(); idx++) {
            cv::drawContours(frame, contoursYel, idx, cv::Scalar(255, 0, 0));
            cv::Moments m = cv::moments(contoursYel[idx]);
            int cX = (int) m.m10 / m.m00;
            int cY = (int) m.m01 / m.m00;
            
            double area = cv::contourArea(contoursYel[idx]);
            double dist = std::sqrt(std::pow(centerX - cX, 2) + std::pow(centerY - cY, 2));
            
            if (area > yelArea)
            {
                yelX = cX;
                yelY = cY;
                yelDist = dist;
            }
            yelArea = std::max(area, yelArea);

            
        }

        if (yelX == -1 && yelY == -1)
        {
            yelX = frame.size().width;
            yelY = frame.size().height/2;
        }

        cv::circle(frame, cv::Point(yelX, yelY), 7, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, "Right", cv::Point(yelX, yelY + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1.5);
        
        cv::arrowedLine(frame, cv::Point(yelX, yelY), cv::Point(centerX, centerY), cv::Scalar(30, 255, 255), static_cast<int>(5 * std::abs(yelDist)/maxDist + 1));

        double rightDist = std::sqrt(std::pow(centerX - yelX, 2) + std::pow(centerY - yelY, 2));
        double leftDist = std::sqrt(std::pow(centerX - redX, 2) + std::pow(centerY - redY, 2));
        double diff = rightDist - leftDist;
        char order = 'S';

        if (diff < diffLeft * -1)
        {
            order = 'L';
            cv::putText(frame, "Left", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);

        }
        else if (diff > diffRight)
        {
            order = 'R';
            cv::putText(frame, "Right", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            order = 'F';
            cv::putText(frame, "Forward", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);
        }

        if (frameCounter % FRAME_RATE == 0)
        {
            std::cout << std::abs(diff) << order << std::endl;
        }

        cv::imshow("Video", frame);
        if (cv::waitKey(33) != -1)
        {
            break;
        }
        frameCounter++;
    }
    cam.release();
    cv::destroyWindow("Video");
    cv::destroyWindow("Config");
    //Save config;
    std::ofstream config;
    config.open("config.txt");
    config << low_HRed << std::endl;
    config << high_HRed << std::endl;
    config << low_SRed << std::endl;
    config << high_SRed << std::endl;
    config << low_VRed << std::endl;
    config << high_VRed << std::endl;
    config << low_HYel << std::endl;
    config << high_HYel << std::endl;
    config << low_SYel << std::endl;
    config << high_SYel << std::endl;
    config << low_VYel << std::endl;
    config << high_VYel << std::endl;
    config << diffLeft << std::endl;
    config << diffRight << std::endl;
    config.close();
    return 0;
}

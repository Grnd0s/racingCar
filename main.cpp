#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

int main(int argc, char *argv[])
{
    raspicam::RaspiCam_Cv cam;
    cam.set( cv::CAP_PROP_FORMAT, CV_8UC1 );
    if (!cam.open())
    {
        std::cerr << "Can't open raspi camera." << std::endl;
        return -1;
    }
    int iLowH = 170;
    int iHighH = 179;

    int iLowS = 150;
    int iHighS = 255;

    int iLowV = 60;
    int iHighV = 255;

    int iLowHY = 30;
    int iHighHY = 40;

    int iLowSY = 100;
    int iHighSY = 255;

    int iLowVY = 100;
    int iHighVY = 255;

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);
    cv::Mat frame;
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
        cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgTreshRed);

        cv::Mat imgTreshYel;
        cv::inRange(imgHSV, cv::Scalar(iLowHY, iLowSY, iLowVY), cv::Scalar(iHighHY, iHighSY, iHighVY), imgTreshYel);
        
        
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
        //std::cout << "dist: " << yelDist << " max dist: " << maxDist  << "dv: " << static_cast<int>(5 * std::abs(yelDist)/maxDist + 1) << std::endl;
        
        cv::arrowedLine(frame, cv::Point(yelX, yelY), cv::Point(centerX, centerY), cv::Scalar(30, 255, 255), static_cast<int>(5 * std::abs(yelDist)/maxDist + 1));

       //std::cout << "Left: " << redDist << " Right: " << yelDist << std::endl;
        double rightDist = std::sqrt(std::pow(centerX - yelX, 2) + std::pow(centerY - yelY, 2));
        double leftDist = std::sqrt(std::pow(centerX - redX, 2) + std::pow(centerY - redY, 2));
        /*if (redDist > 250 && rightDist > 250)
        {
            std::cout << "Forward";
        }
        else if (redDist > 250 && yelDist < 250)
        {
            std::cout << "Left";
        }
        else if (redDist < 250 && yelDist > 250)
        {
            std::cout << "Right";
        }
        std::cout << ": " << ((frame.size().width - redDist) + (frame.size().width - yelDist)) / 100 << std::endl;
        */
        double diff = rightDist - leftDist;
        std::cout << diff;
        if (diff < -50)
        {
            std::cout << "L" << std::endl;
            cv::putText(frame, "Left", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);

        }
        else if (diff > 50)
        {
            std::cout << "R" << std::endl;
            cv::putText(frame, "Right", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            std::cout << "F" << std::endl;
            cv::putText(frame, "Forward", cv::Point(centerX - 50, centerY + 200), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 2);
        }

        cv::imshow("Video", frame);
        if (cv::waitKey(33) != -1)
        {
            break;
        }
    }
    cam.release();
    cv::destroyWindow("Video");
    return 1;
}

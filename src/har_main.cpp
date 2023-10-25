/**
 * @file ar_main.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Main function for the Harris Corners portion of project 4
 * @date 2022-03-24
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "../include/csv_util.h"
#include "../include/ar.h"

int main(int argc, char *argv[]) {
  cv::VideoCapture *capdev;

  // open the video device
  capdev = new cv::VideoCapture(0);
  if( !capdev->isOpened() ) {
    printf("Unable to open video device\n");
    return(-1);
  }

  // get some properties of the image
  cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
  printf("Expected size: %d %d\n", refS.width, refS.height);
  
  cv::namedWindow("Harris Corners", 1); 
  cv::Mat frame;
  cv::Mat dst; 
  int counter = 0; 
  for(;;) {
    *capdev >> frame; // get a new frame from the camera, treat as a stream
    if( frame.empty() ) {
      printf("frame is empty\n");
      break;
    }  

    frame.copyTo(dst);
    // convert frame to gray
    cv::Mat frame_gray; 
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::Mat har_data = cv::Mat::zeros(frame.size(), CV_32FC1);

    cv::cornerHarris(frame_gray, har_data, 2, 3, 0.04); // calculate harris corners
    cv::Mat har_data_norm; 
    cv::normalize(har_data, har_data_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1); // normalize the points

    for( int i = 0; i < har_data_norm.rows ; i++ ) {
      for( int j = 0; j < har_data_norm.cols; j++ ) {
        if( (int) har_data_norm.at<float>(i,j) > 190) {
          cv::circle( dst, cv::Point(j,i), 5, cv::Scalar(255, 0, 0), 2, 8, 0 ); // draw circles when over the threshold
        }
      }
    }
    
    cv::imshow("Harris Corners", dst);

    char keyEx = cv::waitKeyEx(10); 
    if(keyEx == 'q') {
      break; 
    } else if (keyEx == 's') {
      int id = -1; 
      printf("Please enter the id for this image\n"); 
      std::cin >> id; 
      std::string path = "./imgs/image" + std::to_string(id) + ".png"; 
      cv::imwrite(path, dst); 
    }
  }

  printf("Bye!\n"); 
  delete capdev;
  return(0);
}
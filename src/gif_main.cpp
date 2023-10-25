/**
 * @file ar_main.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Main function for the gif extension
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
    printf("Unable to open kermit.gif\n");
    return(-1);
  }
  
   // get some properties of the image
  cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
  printf("Expected size: %d %d\n", refS.width, refS.height);

  cv::namedWindow("Kermit", 1); 
  cv::Mat frame;
  cv::Mat dst; 

  cv::Mat cam_mat(3, 3, CV_64FC1); 
  cv::Mat distcoeff(5, 1, CV_64FC1); 

  read_calibration_data_csv("calibration.csv", cam_mat, distcoeff, 0); 

  // print calibration data
  printf("Camera Matrix\n"); 
  for(int i = 0; i < cam_mat.rows; i++) {
    for(int j = 0; j < cam_mat.cols; j++) {
      printf("%.4f ", cam_mat.at<double>(i, j)); 
    }
    printf("\n"); 
  }
  printf("\n"); 

  printf("Distortion Coefficients\n"); 
  for(int i = 0; i < distcoeff.rows; i++) {
    printf("%.4f ", distcoeff.at<double>(i, 0)); 
  }
  printf("\n\n"); 

  int kermitcount = 0; 

  cv::Size patternsize(9, 6); 

  int counter = 0; 
  for(;;) {
    *capdev >> frame; // get a new frame from the camera, treat as a stream
    if( frame.empty() ) {
      printf("frame is empty\n");
      break;
    }  

    bool patternfound = false;
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> point_set;  
    cv::Mat rotations; 
    cv::Mat translations;
    std::vector<cv::Point2f> image_points; 

    detect_chessboard(frame, patternsize, corner_set, patternfound); 

    frame.copyTo(dst); 

    if(patternfound) {
      get_point_set(patternsize, point_set); // Get the point set for the panner
      cv::solvePnP(point_set, corner_set, cam_mat, distcoeff, rotations, translations);

      printf("Rotations:\n");
      for(int i = 0; i < rotations.rows; i++) {
        printf("%.4f ", rotations.at<double>(i, 0));
      } 
      printf("\n\n"); 

      printf("Translations:\n");
      for(int i = 0; i < translations.rows; i++) {
        printf("%.4f ", rotations.at<double>(i, 0)); 
      }
      printf("\n\n");
      
      // These are the points relative to the origin we are drawing to
      std::vector<cv::Vec3f> drawpoints {
        cv::Vec3f(0, 0, 0), // 0
        cv::Vec3f(9, 0, 0), // 1
        cv::Vec3f(9, -6, 0), // 2
        cv::Vec3f(0, -6, 0) // 3
      }; 

      cv::projectPoints(drawpoints, rotations, translations, cam_mat, distcoeff, image_points);  
      printf("Image Points ( %d )\n[", image_points.size()); 
      for(int i = 0; i < image_points.size(); i++) {
        printf("(%.4f, %.4f) ", image_points[i].x, image_points[i].y); 
      }
      printf("\b]\n\n");
      // read in the kermit image
      std::string fname = "kerm/input-" + std::to_string(kermitcount) + ".png"; 
      cv::Mat kerm = cv::imread(fname); 
      // create points of kermit image
      std::vector<cv::Point2f> kermPoints {
        cv::Point2f(0, 0), 
        cv::Point2f(kerm.cols, 0), 
        cv::Point2f(kerm.cols, kerm.rows), 
        cv::Point2f(0, kerm.rows)
      }; 
      
      cv::Mat h = cv::findHomography(kermPoints, image_points); // find the homography
      cv::Mat warpedKermit; 
      cv::warpPerspective(kerm, warpedKermit, h, frame.size(), cv::INTER_CUBIC); // warp the perspective of the kermit image based on the homography

      // convert the points to integers rather than floats
      std::vector<cv::Point2i> newpoints; 
      for(int i = 0; i < image_points.size(); i++) {
        cv::Point2i newpnt; 
        newpnt.x = (int) image_points[i].x; 
        newpnt.y = (int) image_points[i].y; 
        newpoints.push_back(newpnt); 
      }

      cv::Mat mask; 
      dst.copyTo(mask); 
      for(int i = 0; i < mask.rows; i++) {
        for(int j = 0; j < mask.cols; j++) {
          mask.at<cv::Vec3b>(i, j) = {0, 0, 0}; 
        }
      }
      // fill the mask object with 
      cv::fillConvexPoly(mask, newpoints, cv::Scalar::all(255), cv::LINE_AA); 

      warpedKermit.copyTo(dst, mask); 

      kermitcount++;
      if(kermitcount > 18) kermitcount = 0;  
    }

    cv::imshow("Kermit", dst);

    char keyEx = cv::waitKeyEx(10); 
    if(keyEx == 'q') {
      break; 
    } 
  }

  printf("Bye!\n"); 
  delete capdev;
  return(0);
}
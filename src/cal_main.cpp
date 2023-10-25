/**
 * @file cal_and_ar.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Code to conduct the camera calibration
 * @date 2022-03-16
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "../include/calibration.h"
#include "../include/csv_util.h"

int main(int argc, char *argv[]) {
  cv::VideoCapture *capdev;

  char cal_fn[256] = "calibration.csv"; 
  char rot_fn[256] = "rots.csv"; 
  char tran_fn[256] = "trans.csv"; 

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
  
  cv::namedWindow("Cal/AR", 1); 
  cv::Mat frame;

  // init point_list and corner_list
  std::vector<std::vector<cv::Vec3f> > point_list; 
  std::vector<std::vector<cv::Point2f> > corner_list;  
  cv::Size patternsize(9, 6); 

  // Init various things that go into the Calibrate Camera function
  cv::Mat rotations; 
  cv::Mat translations;
  cv::Mat cam_mat(cv::Size(3, 3), CV_64FC1);
  cam_mat.at<double>(0, 0) = 1.0; cam_mat.at<double>(0, 1) = 0.0; cam_mat.at<double>(0, 2) = (double) frame.cols / 2.0; 
  cam_mat.at<double>(1, 0) = 0.0; cam_mat.at<double>(1, 1) = 1.0; cam_mat.at<double>(1, 2) = (double) frame.rows / 2.0; 
  cam_mat.at<double>(2, 0) = 0.0; cam_mat.at<double>(2, 1) = 0.0; cam_mat.at<double>(2, 2) = 1.0; 
  cv::Mat distcoeff = cv::Mat::zeros(5, 1, CV_64FC1); 
  uchar cal_img_cntr = 0; 
  
  for(;;) {
    *capdev >> frame; // get a new frame from the camera, treat as a stream
    if( frame.empty() ) {
      printf("frame is empty\n");
      break;
    }                 
    std::string cal_img_path = "./cal_imgs/"; 
    cv::Mat dst; 
    frame.copyTo(dst); 

    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> point_set; 
    bool cornersfound = false;

    det_ext_corners(frame, dst, patternsize, corner_set, cornersfound);

    cv::imshow("Cal/AR", dst);

    int keyEx = cv::waitKeyEx(10);
    if(keyEx == 'q')
    {
      break;
    } else if (keyEx == 's') {
      if(!cornersfound) {
        continue; 
      }
      
      printf("Corners (%d):\n", corner_set.size()); 
      for(int i = 0; i < corner_set.size(); i++) {
        printf(" x: %.4f y: %.4f, ", corner_set[i].x, corner_set[i].y); 
      }
      printf("\n\n"); 
      
      // add last corners to corner_list
      corner_list.push_back( std::vector<cv::Point2f>( corner_set )); 
      
      // generate a list of 3d points and add to point_list
      for(int i = 0; i < patternsize.height; i++) {
        for(int j = 0; j < patternsize.width; j++) {
          cv::Vec3f point3d; 
          point3d[0] = j; 
          point3d[1] = -i; 
          point3d[2] = 0;
          point_set.push_back( point3d );  
        }
      }
      
      if(point_set.size() != corner_set.size()) {
        printf("point_set and corner_set not equal\n"); 
        continue; 
      }
      point_list.push_back(std::vector<cv::Vec3f>(point_set));

      // save image 
      std::string name = "cal_img" + std::to_string(cal_img_cntr) + ".png"; 
      std::string fullpath = cal_img_path + name;
      cv::imwrite(fullpath, dst);
      cal_img_cntr++;  

    } else if (keyEx == 'c') {

      if(cal_img_cntr < 5) {
        printf("Not enough images to calibrate\n"); 
        continue; 
      }
      
      printf("Calibrating camera...\n"); 

      printf("Camera Matrix (before):\n"); 
      for(int i = 0; i < cam_mat.rows; i++) {
        for(int j = 0; j < cam_mat.cols; j++) {
          printf("%.4f ", cam_mat.at<double>(i, j)); 
        }
        printf("\n"); 
      }
      printf("\n"); 

      // Calibrate
      double proj_error = cv::calibrateCamera(point_list, corner_list, frame.size(), cam_mat, distcoeff, rotations, translations, cv::CALIB_FIX_ASPECT_RATIO);  
      
      // print camera matrix
      printf("Camera Matrix (after):\n"); 
      for(int i = 0; i < cam_mat.rows; i++) {
        for(int j = 0; j < cam_mat.cols; j++) {
          printf("%.4f ", cam_mat.at<double>(i, j)); 
        }
        printf("\n"); 
      }
      printf("\n"); 

      printf("PROJECTION ERROR: %.4f\n\n", proj_error); 

      // print distortion coefficients
      printf("Distortion Coefficients (%d)\n", distcoeff.rows); 
      for(int i = 0; i < distcoeff.rows; i++){
        printf("%.4f ", distcoeff.at<double>(i, 0));
      }
      printf("\n");

      printf("Rotations:\n"); 
      for(int i = 0; i < rotations.rows; i++) {
        for(int j = 0; j < rotations.cols; j++){
          printf("%d: %.4f ", i, rotations.at<double>(i, j)); 
        }
        printf("\n"); 
      }
      printf("\n\n"); 

      printf("Translations:\n"); 
      for(int i = 0; i < translations.rows; i++) {
        for(int j = 0; j < translations.cols; j++){
          printf("%d: %.4f ", i, translations.at<double>(i, j)); 
        }
        printf("\n"); 
      }
      printf("\n\n"); 

    } else if(keyEx == 'w') {
      printf("Writing to csv...\n"); 
      append_calibration_data_csv(cal_fn, cam_mat, distcoeff, 1);
      printf("Written to csv\n");  
    } else if(keyEx == 'i') {
      int num = -1; 
      printf("Enter the number for the image id\n"); 
      std::cin >> num; 
      std::string path = "./imgs/image" + std::to_string(num) + ".png"; 
      cv::imwrite(path, dst); 
    }
  }

  printf("Bye!\n"); 

  delete capdev;
  return(0);
}
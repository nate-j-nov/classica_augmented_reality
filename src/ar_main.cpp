/**
 * @file ar_main.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Main function for the AR portion of project 4
 * @date 2022-03-22
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
  
  cv::namedWindow("Cal/AR", 1); 
  cv::Mat frame;
  cv::Mat dst; 

  // declare calibration data
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

  // declare size
  cv::Size patternsize(9, 6); 

  bool show_vo = false;
  bool show_ext = false;  

  // get the extension stuff from the object file
  std::map<int, std::vector<float> >  objpoints; 
  std::vector<std::vector<int> > connections; 

  read_vo_data_obj("shuttle.obj", objpoints, connections); 
  
  // print various data about the obj file
  printf("Points (%d):\n", objpoints.size()); 
  for(int i = 1; i <= objpoints.size(); i++) {
    std::vector<float> vect = objpoints[i]; 
    for(int j = 0; j < vect.size(); j++) {
      if(j != vect.size() - 1) {
        printf("%.4f, ", vect[j]); 
      } else {
        printf("%.4f ", vect[j]); 
      }
    }
    printf("\n"); 
  }
  printf("\n\n"); 

  printf("Connections (%d):\n", connections.size()); 
  for(int i = 0; i < connections.size(); i++) {
    std::vector<int> cur = connections[i]; 
    for(int j = 0; j < cur.size(); j++) {
      if(j != cur.size() - 1) {
        printf("%d, ", cur[j]);
      } else {
        printf("%d ", cur[j]);
      }
      
    }
    printf("\n"); 
  }

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
      printf("pattern found\n"); 
      get_point_set(patternsize, point_set); // Get the point set for the panner
      cv::solvePnP(point_set, corner_set, cam_mat, distcoeff, rotations, translations);

      // print results
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

      std::vector<cv::Vec3f> drawpoints;
      if(show_vo) {
        float w = 3.0; 
        float h = 4.0; 
        float d = 5.5; 
        float cenx = 4.5 - 0.5 * w;
        float ceny = -3.0 + 0.5 * h; 
        float cenz = 0; 
        cv::Vec3f origin(cenx, ceny, cenz); 
        draw_rect_prism(drawpoints, origin, w, h, d); // get points for a rectangular prism
        cv::Vec3f rooforig(cenx, ceny, cenz);
        float roofh = 2.0;  
        draw_roof(drawpoints, rooforig, w, roofh, d); // get points for the roof
        cv::Vec3f doororig(4.5 - .25 * w, ceny  - h, cenz);
        draw_door(drawpoints, doororig, 0.25 * w, 0.25 * h, d);  // get points for the door
      } else if(show_ext) {
        float cenx = 4.5; 
        float ceny = -3.0; 
        float cenz = 1.0; 
        // create 
        for(int i = 1; i <= objpoints.size(); i++) {
          cv::Vec3f npoint; 
          npoint[0] = cenx + objpoints[i][0]; 
          npoint[1] = ceny + objpoints[i][1]; 
          npoint[2] = cenz + objpoints[i][2]; 
          drawpoints.push_back(npoint); 
        }
      } else {
        draw_axes(drawpoints, cv::Vec3f(0, 0, 0), 1);
      }
      
      // project the points and get the image points  
      cv::projectPoints(drawpoints, rotations, translations, cam_mat, distcoeff, image_points);  
      
      // uncomment for debugging
      /*printf("Image Points ( %d )\n[", image_points.size()); 
      for(int i = 0; i < image_points.size(); i++) {
        printf("(%.4f, %.4f) ", image_points[i].x, image_points[i].y); 
      }
      printf("\b]\n\n");*/
      
      if(show_vo) {
        // rectangle
        // 0 -> 1
        cv::line(dst, image_points[0], image_points[1], {255, 0, 0}, 2); 
        // 1 -> 2
        cv::line(dst, image_points[1], image_points[2], {255, 0, 0}, 2); 
        // 2 -> 3 
        cv::line(dst, image_points[2], image_points[3], {255, 0, 0}, 2); 
        // 3 -> 0 
        cv::line(dst, image_points[3], image_points[0], {255, 0, 0}, 2); 
        // 4 -> 5
        cv::line(dst, image_points[4], image_points[5], {255, 0, 0}, 2); 
        // 5 -> 6
        cv::line(dst, image_points[5], image_points[6], {255, 0, 0}, 2); 
        // 6 -> 7
        cv::line(dst, image_points[6], image_points[7], {255, 0, 0}, 2); 
        // 7 -> 4
        cv::line(dst, image_points[7], image_points[4], {255, 0, 0}, 2); 
        // 0 -> 4
        cv::line(dst, image_points[0], image_points[4], {255, 0, 0}, 2); 
        // 1 -> 5
        cv::line(dst, image_points[1], image_points[5], {255, 0, 0}, 2); 
        // 2 -> 6
        cv::line(dst, image_points[2], image_points[6], {255, 0, 0}, 2); 
        // 3 -> 7  
        cv::line(dst, image_points[3], image_points[7], {255, 0, 0}, 2); 

        // roof
        // 8 -> 9
        cv::line(dst, image_points[8], image_points[9], {0, 0, 255}, 2); 
        // 9 -> 10
        cv::line(dst, image_points[9], image_points[10], {0, 0, 255}, 2); 
        // 10 -> 8
        cv::line(dst, image_points[10], image_points[8], {0, 0, 255}, 2); 
        // 11 -> 12
        cv::line(dst, image_points[11], image_points[12], {0, 0, 255}, 2); 
        // 12 -> 13
        cv::line(dst, image_points[12], image_points[13], {0, 0, 255}, 2); 
        // 13 -> 11
        cv::line(dst, image_points[13], image_points[11], {0, 0, 255}, 2);
        // 8 -> 11
        cv::line(dst, image_points[8], image_points[11], {0, 0, 255}, 2); 
        // 9 -> 12
        cv::line(dst, image_points[9], image_points[12], {0, 0, 255}, 2); 
        // 10 -> 13
        cv::line(dst, image_points[10], image_points[13], {0, 0, 255}, 2); 

        //door
        // 14 -> 15
        cv::line(dst, image_points[14], image_points[15], {0, 0, 0}, 2); 
        // 15 -> 16
        cv::line(dst, image_points[15], image_points[16], {0, 0, 0}, 2);
        // 16 -> 17
        cv::line(dst, image_points[16], image_points[17], {0, 0, 0}, 2);
        // 17 -> 14
        cv::line(dst, image_points[17], image_points[14], {0, 0, 0}, 2);
        // doorknob
        cv::circle(dst, image_points[18], 2, {0, 0, 0}, 3); 
      } else if(show_ext) {
        // put back in a map for easy organization
        // given the .obj file. May be overkill on second thought
        std::map<int, cv::Point2f> pointmap; 
        for(int i = 0; i < image_points.size(); i++) {
          pointmap[i + 1] = image_points[i];  
        }

        // loop through the connections array
        for(int i = 0; i < connections.size(); i++) {
          std::vector<int> curvec = connections[i]; // get current
          int first = curvec[0]; // record the first
          int prev = -1; // init prev
          for(int j = 1; j < curvec.size(); j++) { // loop through the current vec of connections
            int current = curvec[j]; // set current j
            if(j == 1) { // if j == 1, then previous is the first point
              prev = curvec[0]; 
            }
            // the last point connects to the first point
            if(j == curvec.size() - 1) {
              cv::line(dst, pointmap[current], pointmap[first], {255, 0, 0}, 1); 
            } else {
              cv::line(dst, pointmap[current], pointmap[prev], {255, 0, 0,}, 1); // otherwise, connect current with previous
              prev = current; // update previous
            }
          }
        }
      } else {
        cv::arrowedLine(dst, image_points[0], image_points[1], {255, 0, 0}, 2); // z
        cv::arrowedLine(dst, image_points[0], image_points[2], {0, 255, 0}, 2); // y
        cv::arrowedLine(dst, image_points[0], image_points[3], {0, 0, 255}, 2); // x
      }  
    }

    cv::imshow("Cal/AR", dst);

    char keyEx = cv::waitKeyEx(10); 
    if(keyEx == 'q') {
      break; 
    } else if (keyEx == 'n') {
      show_vo = !show_vo;
      show_ext = false; 
    } else if (keyEx == 'e') {
      show_ext = !show_ext; 
      show_vo = false; 
    } else if (keyEx == 's') {
      int id = -1; 
      printf("What number do you want to assign this image?\n");  
      std::cin >> id; 
      std::string name = "./imgs/image" + std::to_string(id) + ".png"; 
      cv::imwrite(name, dst); 
    } 
  }

  printf("Bye!\n"); 

  delete capdev;
  return(0);
}
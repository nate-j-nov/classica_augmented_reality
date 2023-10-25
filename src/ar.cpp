/**
 * @file ar.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Library of functions for conducting augmented reality
 * @date 2022-03-23
 */ 
#include "../include/ar.h"

/**
 * @brief Function to detect and extract chessboard
 * 
 * @param src source image to find the corners in 
 * @param dst dst image that displays the the corners
 * @param patsize size of the pattern
 * @param corner_set vector of the point location of each corner  
 * @param pattern_found bool passed by reference to determine if corners were found. 
 * @return int return non-zero value on failure. 
 */
int detect_chessboard(const cv::Mat &src, cv::Size patsize, std::vector<cv::Point2f> &corner_set, bool &pattern_found) { 
  pattern_found = cv::findChessboardCorners(src, patsize, corner_set, cv::CALIB_CB_FAST_CHECK); 

  if(pattern_found) {
    cv::Mat gray; 
    cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY); 
    cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
  }   
  return 0; 
} 

/**
 * @brief Function to get the point set if there's a pattern
 * 
 * @param pat_size size of the pattern
 * @param point_set point set to write to
 * @return int N/A
 */
int get_point_set(const cv::Size pat_size, std::vector<cv::Vec3f> &point_set) {
  
  for(int i = 0; i < pat_size.height; i++) {
    for(int j = 0; j < pat_size.width; j++) {
      cv::Vec3f point3d; 
      point3d[0] = j; 
      point3d[1] = -i; 
      point3d[2] = 0; 
      point_set.push_back( point3d ); 
    }
  }

  return 0; 
}

/**
 * @brief Function to draw a cube 
 * 
 * @param points vector of world points to write to
 * @param origin origin of the cube
 * @param scale scale of the cube
 * @param color color of the cube
 * @return int 
 */
int draw_axes(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float scale){
  points.push_back(cv::Vec3f(0, 0, 0)); // origin 
  points.push_back(cv::Vec3f(0, 0, 1)); // z axis
  points.push_back(cv::Vec3f(1, 0, 0)); // x axis
  points.push_back(cv::Vec3f(0, 1, 0)); // y axis

  return 0; 
} 

/**
 * @brief Function to draw a cube 
 * 
 * @param points point list to write to 
 * @param origin origin of the cube
 * @param scale scale of the cube
 * @param color color of the cube
 * @return int 
 */
int draw_cube(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float scale) {
  // get origin points
  float xo = origin[0]; 
  float yo = origin[1]; 
  float zo = origin[2];
  
  // init points 
  cv::Vec3f p000(xo + scale * 0, yo - scale * 0, zo + scale * 0); //0
  points.push_back(p000); 
  cv::Vec3f p100(xo + scale * 1, yo - scale * 0, zo + scale * 0); //1
  points.push_back(p100); 
  cv::Vec3f p101(xo + scale * 1, yo - scale * 0, zo + scale * 1); //2
  points.push_back(p101); 
  cv::Vec3f p001(xo + scale * 0, yo - scale * 0, zo + scale * 1); //3
  points.push_back(p001); 
  cv::Vec3f p010(xo + scale * 0, yo - scale * 1, zo + scale * 0); //4
  points.push_back(p010); 
  cv::Vec3f p110(xo + scale * 1, yo - scale * 1, zo + scale * 0); //5
  points.push_back(p110); 
  cv::Vec3f p111(xo + scale * 1, yo - scale * 1, zo + scale * 1); //6
  points.push_back(p111);  
  cv::Vec3f p011(xo + scale * 0, yo - scale * 1, zo + scale * 1); //7
  points.push_back(p011);  

  return 0; 
}

/**
 * @brief Function to draw a cube 
 * 
 * @param points point list to write to 
 * @param origin origin of the cube
 * @param w width
 * @param h height
 * @param d depth
 * @return int 
 */
int draw_rect_prism(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d) {
  // get origin points
  float xo = origin[0]; 
  float yo = origin[1]; 
  float zo = origin[2];
  
  // init points 
  cv::Vec3f p000(xo + w * 0, yo - h * 0, zo + d * 0); //0
  points.push_back(p000); 
  cv::Vec3f p100(xo + w * 1, yo - h * 0, zo + d * 0); //1
  points.push_back(p100); 
  cv::Vec3f p101(xo + w * 1, yo - h * 0, zo + d * 1); //2
  points.push_back(p101); 
  cv::Vec3f p001(xo + w * 0, yo - h * 0, zo + d * 1); //3
  points.push_back(p001); 
  cv::Vec3f p010(xo + w * 0, yo - h * 1, zo + d * 0); //4
  points.push_back(p010); 
  cv::Vec3f p110(xo + w * 1, yo - h * 1, zo + d * 0); //5
  points.push_back(p110); 
  cv::Vec3f p111(xo + w * 1, yo - h * 1, zo + d * 1); //6
  points.push_back(p111);  
  cv::Vec3f p011(xo + w * 0, yo - h * 1, zo + d * 1); //7
  points.push_back(p011);  

  return 0; 
}

/**
 * @brief Function to draw a rooftop for my house
 * 
 * @param points points to write to
 * @param origin origin for where to start drawing 
 * @param w width
 * @param h height 
 * @param d depth
 * @return int 
 */
int draw_roof(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d) {
  // get origin points
  float xo = origin[0]; 
  float yo = origin[1]; 
  float zo = origin[2];

  // back triangle
  cv::Vec3f p000(xo + w * 0, yo + h * 0, zo + d * 0); // 8
  points.push_back(p000); 
  cv::Vec3f ph10(xo + w * 0.5, yo + h * 1, zo + d * 0); // 9
  points.push_back(ph10); 
  cv::Vec3f p100(xo + w * 1, yo + h * 0, zo + d * 0); // 10
  points.push_back(p100); 
  // front triangle
  cv::Vec3f p001(xo + w * 0, yo + h * 0, zo + d * 1); // 11
  points.push_back(p001); 
  cv::Vec3f ph11(xo + w * 0.5, yo + h * 1, zo + d * 1); // 12
  points.push_back(ph11); 
  cv::Vec3f p101(xo + w * 1, yo + h * 0, zo + d * 1); // 13
  points.push_back(p101);

  return 0;
}

/**
 * @brief Function to draw a door 
 * 
 * @param points point list to write to
 * @param origin origin of the door
 * @param w width
 * @param h height
 * @param d depth
 * @return int 
 */
int draw_door(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d) {
  // get origin points
  float xo = origin[0]; 
  float yo = origin[1]; 
  float zo = origin[2];

  // Frame
  cv::Vec3f p00(xo + w * 0, yo + h * 0, zo + d * 1); // 14
  points.push_back(p00);
  cv::Vec3f p01(xo + w * 0, yo + h * 1, zo + d * 1); // 15
  points.push_back(p01);
  cv::Vec3f p11(xo + w * 1, yo + h * 1, zo + d * 1); // 16
  points.push_back(p11);
  cv::Vec3f p10(xo + w * 1, yo + h * 0, zo + d * 1); // 17
  points.push_back(p10);

  // knob loc
  cv::Vec3f knob(xo + w * 0.2, yo + h * 0.6, zo + d * 1); // 18
  points.push_back(knob); 
  return 0; 
}
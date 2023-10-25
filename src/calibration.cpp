/**
 * @file calibration.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Collection of functions to calibrate the camera
 * @date 2022-03-16
 */

#include "../include/calibration.h"

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
int det_ext_corners(const cv::Mat &src, cv::Mat &dst, cv::Size patsize, std::vector<cv::Point2f> &corner_set, bool &pattern_found) { 
  pattern_found = cv::findChessboardCorners(src, patsize, corner_set, cv::CALIB_CB_FAST_CHECK); 

  if(pattern_found) {
    cv::Mat gray; 
    cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY); 
    cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));  
  }

  cv::drawChessboardCorners(dst, patsize, corner_set, pattern_found);

  
   
  return 0; 
} 
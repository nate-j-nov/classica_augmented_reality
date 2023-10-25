/**
 * @file calibration.cpp
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Header file for calibration.cpp
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
int det_ext_corners(const cv::Mat &src, cv::Mat &dst, cv::Size patsize, std::vector<cv::Point2f> &corner_set, bool &pattern_found); 
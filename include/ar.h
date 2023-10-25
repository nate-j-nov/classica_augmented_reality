/**
 * @file ar.h
 * @author Nate Novak (novak.n@northeastern.edu)
 * @brief Header file for ar.cpp
 * @date 2022-03-23
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
int detect_chessboard(const cv::Mat &src, cv::Size patsize, std::vector<cv::Point2f> &corner_set, bool &pattern_found);

/**
 * @brief Function to get the point set if there's a pattern
 * 
 * @param pat_size size of the pattern
 * @param point_set point set to write to
 * @return int 
 */
int get_point_set(const cv::Size pat_size, std::vector<cv::Vec3f> &point_set); 

/**
 * @brief Function to draw a cube 
 * 
 * @param points vector of world points to write to
 * @param origin origin of the cube
 * @param scale scale of the cube
 * @param color color of the cube
 * @return int 
 */
int draw_axes(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float scale); 

/**
 * @brief Function to draw a cube 
 * 
 * @param points vector of world points to write to
 * @param origin origin of the cube
 * @param scale scale of the cube
 * @param color color of the cube
 * @return int 
 */
int draw_cube(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float scale); 

/**
 * @brief Function to draw a cube 
 * 
 * @param points point list to write to 
 * @param origin origin of the cube
 * @param scale scale of the cube
 * @param color color of the cube
 * @return int 
 */
int draw_rect_prism(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d); 

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
int draw_roof(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d);

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
int draw_door(std::vector<cv::Vec3f> &points, cv::Vec3f origin, float w, float h, float d); 
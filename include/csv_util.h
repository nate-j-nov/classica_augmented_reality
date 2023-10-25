/*
  Bruce A. Maxwell

  Utility functions for reading and writing CSV files with a specific format

  Each line of the csv file is a filename in the first column, followed by numeric data for the remaining columns
  Each line of the csv file has to have the same number of columns
 */


#ifndef CVS_UTIL_H
#define CVS_UTIL_H

#include <vector>
/*
  Given a filename, and image filename, and the image features, by
  default the function will append a line of data to the CSV format
  file.  If reset_file is true, then it will open the file in 'write'
  mode and clear the existing contents.

  The image filename is written to the first position in the row of
  data. The values in image_data are all written to the file as
  floats.

  The function returns a non-zero value in case of an error.
 */
int append_image_data_csv( char *filename, std::vector<float> &image_data, int reset_file = 0 );

/* Function to write image data to a csv file
  parameter file name is the name of the csv file
  parameter image_file_name is the name of the image
  parameter image_data is the mat representing the histogram for the image 
  parameter reset_file indicates whether to clear the file
*/
int veci_append_image_data_csv(char *filename, std::vector<int> &image_data, int reset_file = 0); 

/*
  Given a file with the format of a string as the first column and
  floating point numbers as the remaining columns, this function
  returns the filenames as a std::vector of character arrays, and the
  remaining data as a 2D std::vector<float>.

  filenames will contain all of the image file names.
  data will contain the features calculated from each image.

  If echo_file is true, it prints out the contents of the file as read
  into memory.

  The function returns a non-zero value if something goes wrong.
 */
int read_image_data_csv( char *filename, std::vector<std::vector<float>> &data, int echo_file = 0 );

int veci_read_image_data_csv( char *filename, std::vector<char *> &filenames, std::vector<std::vector<int>> &data, int echo_file = 0); 

/**
 * @brief Function to write calibration data to a csv 
 * 
 * @param filename name of csv file
 * @param cam_mat camera matrix
 * @param distcoeff distance coefficients
 * @param reset_file indicator whether to reset the file or not
 * @return int return non-zero value on failure
 */
int append_calibration_data_csv( char *filename, cv::Mat &cam_mat, cv::Mat &distcoeff, int reset_file ); 

/**
 * @brief Function to write rotation and translation data to a csv 
 * 
 * @param filename name of csv file
 * @param rot_or_trans rotation or translation data
 * @param reset_file indicator whether to reset the file or not
 * @return int return non-zero value on failure
 */
int append_rot_trans_data_csv( char *filename, std::vector<std::vector<float> > rot_or_trans, int reset_file ); 

/**
 * @brief Function to read the calibration data from a csv file. 
 * 
 * @param filename 
 * @param cam_mat camera matrix to write to. 
 * @param distcoeff distance coeffcients to write to
 * @param echo_file if true, print the data. 
 * @return int 
 */
int read_calibration_data_csv( char *filename, cv::Mat &cam_mat, cv::Mat &distcoeff, int echo_file );

/**
 * @brief Function to read the virtual object data from a csv file. 
 * 
 * @param filename 
 * @param points vector of points to write to
 * @param connections indicates the points to connect
 * @param echo_file if true, print the data. 
 * @return int 
 */
int read_vo_data_obj( char *filename, std::map<int, std::vector<float> > &points, std::vector<std::vector<int> > &connections);
#endif

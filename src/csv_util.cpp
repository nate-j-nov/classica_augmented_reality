/*
Bruce A. Maxwell

EDITED BY NATE NOVAK

CS 5330 Computer Vision
Spring 2021

CPP functions for reading CSV files with a specific format
- first column is a string containing a filename or path
- every other column is a number

The function returns a std::vector of char* for the filenames and a 2D std::vector of floats for the data
*/

#include <cstdio>
#include <cstring>
#include <vector>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "../include/csv_util.h"

/*
  reads a string from a CSV file. the 0-terminated string is returned in the char array os.

  The function returns false if it is successfully read. It returns true if it reaches the end of the line or the file.
 */
int getstring( FILE *fp, char os[] ) {
  int p = 0;
  int eol = 0;
  
  for(;;) {
    char ch = fgetc( fp );
    if( ch == ',' ) {
      break;
    }
    else if( ch == '\n' || ch == EOF ) {
      eol = 1;
      break;
    }
    //printf("%c", ch ); // uncomment for debugging
    os[p] = ch;
    p++;
  }
   printf("\n"); // uncomment for debugging
  os[p] = '\0';

  return(eol); // return true if eol
}

/*
  reads a string from a CSV file. the 0-terminated string is returned in the char array os.

  The function returns false if it is successfully read. It returns true if it reaches the end of the line or the file.
 */
int getstring_obj( FILE *fp, char os[] ) {
  int p = 0;
  int eol = 0;
  
  for(;;) {
    char ch = fgetc( fp );
    if( ch == ' ') {
      break;
    }
    else if( ch == '\n' || ch == EOF ) {
      eol = 1;
      break;
    } 
    //printf("%c\n", ch ); // uncomment for debugging
    os[p] = ch; 
    p++;
    
  }
  //printf("\n"); // uncomment for debugging
  os[p] = '\0';
  //printf("%s\n", os); 

  return(eol); // return true if eol
}

int getint(FILE *fp, int *v) {
  char s[256];
  int p = 0;
  int eol = 0;

  for(;;) {
    char ch = fgetc( fp );
    if( ch == ',') {
      break;
    }
    else if(ch == '\n' || ch == EOF) {
      eol = 1;
      break;
    }
      
    s[p] = ch;
    p++;
  }
  s[p] = '\0'; // terminator
  *v = atoi(s);

  return(eol); // return true if eol
}

int getint_obj(FILE *fp, int *v) {
  char s[256];
  int p = 0;
  int eol = 0;

  for(;;) {
    char ch = fgetc( fp );
    if( ch == ' ') {
      break;
    }
    else if(ch == '\n' || ch == EOF) {
      eol = 1;
      break;
    }
      
    s[p] = ch;
    p++;
  }
  s[p] = '\0'; // terminator
  *v = atoi(s);

  return(eol); // return true if eol
}

/*
  Utility function for reading one float value from a CSV file

  The value is stored in the v parameter

  The function returns true if it reaches the end of a line or the file
 */
int getfloat(FILE *fp, float *v) {
  char s[256];
  int p = 0;
  int eol = 0;

  for(;;) {
    char ch = fgetc( fp );
    if( ch == ',') {
      break;
    }
    else if(ch == '\n' || ch == EOF) {
      eol = 1;
      break;
    }
      
    s[p] = ch;
    p++;
  }
  s[p] = '\0'; // terminator
  *v = atof(s);

  return(eol); // return true if eol
}

/*
  Utility function for reading one float value from a obj file

  The value is stored in the v parameter

  The function returns true if it reaches the end of a line or the file
 */
int getfloat_obj(FILE *fp, float *v) {
  char s[256];
  int p = 0;
  int eol = 0;

  for(;;) {
    char ch = fgetc( fp );
    if( ch == ' ') {
      break;
    }
    else if(ch == '\n' || ch == EOF) {
      eol = 1;
      break;
    }
      
    s[p] = ch;
    p++;
  }
  s[p] = '\0'; // terminator
  *v = atof(s);

  return(eol); // return true if eol
}

/*
  Utility function for reading one float value from a CSV file

  The value is stored in the v parameter

  The function returns true if it reaches the end of a line or the file
 */
int getdouble(FILE *fp, double *v) {
  char s[256];
  int p = 0;
  int eol = 0;

  for(;;) {
    char ch = fgetc( fp );
    if( ch == ',') {
      break;
    }
    else if(ch == '\n' || ch == EOF) {
      eol = 1;
      break;
    }
      
    s[p] = ch;
    p++;
  }
  s[p] = '\0'; // terminator
  *v = atof(s);

  return(eol); // return true if eol
}

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
int append_image_data_csv( char *filename, std::vector<float> &image_data, int reset_file ) {
  char mode[8];
  FILE *fp;
  
  strcpy(mode, "a");

  if( reset_file ) {
    strcpy( mode, "w" );
  }
  
  fp = fopen( filename, mode );
  if(!fp) {
    printf("Unable to open output file %s\n", filename );
    exit(-1);
  }

  // write the filename and the feature vector to the CSV file
  for(int i=0;i<image_data.size();i++) {
    char tmp[256];
    if(i == image_data.size() - 1) {
      sprintf(tmp, "%.4f", image_data[i] );
    } else {
      sprintf(tmp, "%.4f,", image_data[i] );
    }
    std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
  }
      
  std::fwrite("\n", sizeof(char), 1, fp); // EOL

  fclose(fp);
  
  return(0);
}


/* Function to write image data to a csv file
  parameter file name is the name of the csv file
  parameter image_file_name is the name of the image
  parameter image_data is the mat representing the histogram for the image 
  parameter reset_file indicates whether to clear the file
*/
int veci_append_image_data_csv(char *filename, std::vector<int> &image_data, int reset_file) {
  ///char buffer[256];
  char mode[8];
  FILE *fp;
  
  strcpy(mode, "a");

  if( reset_file ) {
    strcpy( mode, "w" );
  }
  
  fp = fopen( filename, mode );
  if(!fp) {
    printf("Unable to open output file %s\n", filename );
    exit(-1);
  }

  // write the filename and the feature vector to the CSV file
  //strcpy(buffer, image_filename);
  //std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
  for(int i=0;i<image_data.size();i++) {
    char tmp[256];
    if(i == image_data.size() - 1) {
      sprintf(tmp, "%d", image_data[i]);
    } else {
      sprintf(tmp, "%d,", image_data[i]);
    }
    std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
  }
      
  std::fwrite("\n", sizeof(char), 1, fp); // EOL

  fclose(fp);
  
  return(0);
}

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
int read_image_data_csv( char *filename, std::vector<std::vector<float>> &data, int echo_file ) {
  FILE *fp;
  float fval;

  fp = fopen(filename, "r");
  if( !fp ) {
    printf("Unable to open feature file\n");
    return(-1);
  }

  printf("Reading %s\n", filename);
  for(;;) {
    std::vector<float> dvec;
    
    // read the filename
    if( getfloat( fp, &fval ) ) {
      break;
    }
    // printf("Evaluting %s\n", filename);
    dvec.push_back(fval); 

    // read the whole feature file into memory
    for(;;) {
      // get next feature
      float eol = getfloat( fp, &fval );
      dvec.push_back( fval );
      if( eol ) break;
    }
    // printf("read %lu features\n", dvec.size() );

    data.push_back(dvec);

  }
  fclose(fp);
  printf("Finished reading CSV file\n");

  if(echo_file) {
    for(int i=0;i<data.size();i++) {
      for(int j=0;j<data[i].size();j++) {
	printf("%.4f  ", data[i][j] );
      }
      printf("\n");
    }
    printf("\n");
  }

  return(0);
}

/*
  Given a file with the format of a string as the first column and
  integers as the remaining columns, this function
  returns the filenames as a std::vector of character arrays, and the
  remaining data as a 2D std::vector<int>.

  filenames will contain all of the image file names.
  data will contain the features calculated from each image.

  If echo_file is true, it prints out the contents of the file as read
  into memory.

  The function returns a non-zero value if something goes wrong.
 */
int veci_read_image_data_csv( char *filename, std::vector<char *> &filenames, std::vector<std::vector<int>> &data, int echo_file ) {
  FILE *fp;
  int fval;

  fp = fopen(filename, "r");
  if( !fp ) {
    printf("Unable to open feature file\n");
    return(-1);
  }

  printf("Reading %s\n", filename);
  for(;;) {
    std::vector<int> dvec;
    
    // read the filename
    if( getstring( fp, filename ) ) {
      break;
    }
    // printf("Evaluting %s\n", filename);

    // read the whole feature file into memory
    for(;;) {
      // get next feature
      int eol = getint( fp, &fval );
      dvec.push_back( fval );
      if( eol ) break;
    }
    // printf("read %lu features\n", dvec.size() );

    data.push_back(dvec);

    char *fname = new char[strlen(filename)+1];
    strcpy(fname, filename);
    filenames.push_back( fname );
  }
  fclose(fp);
  printf("Finished reading CSV file\n");

  if(echo_file) {
    for(int i=0;i<data.size();i++) {
      for(int j=0;j<data[i].size();j++) {
	printf("%d  ", data[i][j] );
      }
      printf("\n");
    }
    printf("\n");
  }

  return(0);
}

/**
 * @brief Function to write calibration data to a csv 
 * 
 * @param filename name of csv file
 * @param cam_mat camera matrix
 * @param distcoeff distance coefficients
 * @param reset_file indicator whether to reset the file or not
 * @return int return non-zero value on failure
 */
int append_calibration_data_csv( char *filename, cv::Mat &cam_mat, cv::Mat &distcoeff, int reset_file ) {
  char mode[8];
  FILE *fp;
  
  strcpy(mode, "a");

  if( reset_file ) {
    strcpy( mode, "w" );
  }
  
  fp = fopen( filename, mode );
  if(!fp) {
    printf("Unable to open output file %s\n", filename );
    exit(-1);
  }

  char buf[256]; 
  sprintf(buf, "cm00 cm01 cm02 cm10 cm11 cm12 cm20 cm21 cm22 dist1 dist2 dist3 dist4 dist5\n"); 
  std::fwrite(buf, sizeof(char), strlen(buf), fp); 

  // write the camera matrix to the file
  for(int i=0;i<cam_mat.rows; i++) {
    for(int j = 0; j < cam_mat.cols; j++) {
      char tmp[256];
      sprintf(tmp, "%.4f,", cam_mat.at<double>(i, j));
      std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
    }
  }

  // write the distcoeff to the file
  for(int i = 0; i < distcoeff.rows; i++) {
    char tmp[256];
    if(i != distcoeff.rows - 1) {
      sprintf(tmp, "%.4f,", distcoeff.at<double>(i, 0)); 
    } else {
      sprintf(tmp, "%.4f", distcoeff.at<double>(i, 0)); 
    }
    std::fwrite(tmp, sizeof(char), strlen(tmp), fp); 
  }
      
  std::fwrite("\n", sizeof(char), 1, fp); // EOL

  fclose(fp);
  
  return(0);
}

/**
 * @brief Function to write rotation and translation data to a csv 
 * 
 * @param filename name of csv file
 * @param rot_or_trans rotation or translation data
 * @param reset_file indicator whether to reset the file or not
 * @return int return non-zero value on failure
 */
int append_rot_trans_data_csv( char *filename, std::vector<std::vector<float> > rot_or_trans, int reset_file ) {
  char mode[8];
  FILE *fp;
  
  strcpy(mode, "a");

  if( reset_file ) {
    strcpy( mode, "w" );
  }
  
  fp = fopen( filename, mode );
  if(!fp) {
    printf("Unable to open output file %s\n", filename );
    exit(-1);
  }

  int size = rot_or_trans[0].size() - 1; 

  // write the camera matrix to the file
  for(int i=0;i<rot_or_trans.size();i++) {
    for(int j = 0; j < rot_or_trans[i].size(); j++) {
      char tmp[256];
      if(j == rot_or_trans[i].size() - 1) {
        sprintf(tmp, ".4f", rot_or_trans[i][j]);
      } else {
        sprintf(tmp, "%.4f,", rot_or_trans[i][j]);
      }
      std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
    }
    std::fwrite("\n", sizeof(char), strlen("\n"), fp ); 
  }

  std::fwrite("\n", sizeof(char), 1, fp); // EOL

  fclose(fp);
  
  return(0);
}

/**
 * @brief Function to read the calibration data from a csv file. 
 * 
 * @param filename 
 * @param cam_mat camera matrix to write to. 
 * @param distcoeff distance coeffcients to write to
 * @param echo_file if true, print the data. 
 * @return int 
 */
int read_calibration_data_csv( char *filename, cv::Mat &cam_mat, cv::Mat &distcoeff, int echo_file ) {
  FILE *fp;
  double fval;

  fp = fopen(filename, "r");
  if( !fp ) {
    printf("Unable to open feature file\n");
    return(-1);
  }

  printf("Reading %s\n", filename);
  int linenum = 0;
  int cellnum = -1; 
  int distcellnum = 0; 
  std::vector<double> test; 

  for(;;) {
    if( linenum == 0 ) {
      linenum++; 
      continue;   
    }

    // printf("Evaluting %s\n", filename); 

    // read the whole feature file into memory
    for(;;) {
      double eol = getdouble( fp, &fval );
      if(cellnum == -1) {
        cellnum++;        
        continue;
      }
      if(cellnum < 9) {
        int i = (cellnum / 3); 
        int j = (cellnum % 3);
        cam_mat.at<double>(i, j) = fval;  
      }
      else { 
        distcoeff.at<double>(distcellnum, 0) = fval;
        distcellnum++;  
      }
      test.push_back(fval); 
      cellnum++;
      // get next feature
      if( cellnum == 14 ) break;
    }
    // printf("read %lu features\n", dvec.size() );
    if(linenum==1) break; 
    linenum++; 
  }
  fclose(fp);
  printf("Finished reading CSV file\n\n");
  return(0);
}

/**
 * @brief Function to read the virtual object data from a csv file. 
 * 
 * @param filename 
 * @param points vector of points to write to
 * @param connections indicates the points to connect
 * @param echo_file if true, print the data. 
 * @return int 
 */
int read_vo_data_obj( char *filename, std::map<int, std::vector<float> > &points, std::vector<std::vector<int> > &connections) {
  FILE *fp;
  char indicator[8]; 
  float fval;
  int dval; 

  fp = fopen(filename, "r");
  if( !fp ) {
    printf("Unable to open feature file\n");
    return(-1);
  }

  printf("Reading %s\n", filename);
  int pointcounter = 1; 
  for(;;) {
    std::vector<float> fvec;
    std::vector<int> dvec; 
    // read the filename
    if( getstring_obj( fp, indicator ) ) {
      break;
    } 
    bool point = false;
    bool conn = false;

    // if it's a point, set point to true, else false (meaning these are connections between points)
    if( std::strcmp("v\0", indicator) == 0 ) {
      point = true; 
    } else if( std::strcmp("f\0", indicator) == 0 ) {
      conn = true;
    } else {
      continue; 
    }
    
    // printf("Evaluting %s\n", filename);

    // read the whole feature file into memory
    for(;;) {
      if(point) {
        int eol = getfloat_obj(fp, &fval); 
        fvec.push_back(fval); 
        if( eol ) break; 
      } 
      if(conn) {
        int eol = getint_obj( fp, &dval );
        //printf("%d ", dval); 
        dvec.push_back( dval );
        if( eol ) break;
      }

      // get next feature
      
    } 
    //printf("read %lu features\n", dvec.size() );

    if(point) {
      points[pointcounter] = fvec; 
      pointcounter++; 
    } else {
      connections.push_back( dvec ); 
    }
  }
  fclose(fp);
  printf("Finished reading OBJ file\n");
  return(0);
}

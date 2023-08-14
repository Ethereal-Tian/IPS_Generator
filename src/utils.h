#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <assert.h>

#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "PFM_ReadWrite/PFMReadWrite.h"

using namespace std;
using namespace cv;

/* =======================================
 * ============= File IO =================
 * ======================================= */

std::vector<std::string> getFileNames(const std::string dir_path, std::string prefix="", std::string suffix="");

void checkAndCreateDir(const std::string DirPath);

/* =======================================
 * ========= Math Calculation ============
 * ======================================= */

// quartic equation solver

std::vector<float> solveQuartic(float a,float b,float c,float d,float e);

float solveQuadratic(float a,float b,float c);

float diffuseZenithToRho(float zenith,float eta);

float specularZenithToRho(float zenith,float eta);

float diffuseRhoToZenith(float rho,float eta);

std::vector<float> specularRhoToZenith(float rho,float eta);


float diffuseAzimuthToPhi(float azimuth);

float specularAzimuthToPhi(float azimuth);

std::vector<float> diffusePhiToAzimuth(float phi);

std::vector<float> specularPhiToAzimuth(float phi);



/* =======================================
 * ============= Image Show ==============
 * ======================================= */

cv::Vec3b Normal2Color(cv::Vec3f normValue);

char DoLP2Gray(float dolpValue);

cv::Vec3b DAoLP2Color(float dolpValue,float aolpValue);

cv::Vec3b AoLP2Color(float aolpValue);

void showNormal(const cv::Mat normal, std::string name);

void visualizeNormal(const cv::Mat normal_img, const bool show, const string path);

void showDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string suffix);

void showColorMap();

/* =======================================
 * ============== Image IO ===============
 * ======================================= */

void outputImage(const cv::Mat& img);

void saveVisualizedOneChannelImage(
    const cv::Mat img, 
    const std::string path,
    const float min_value,
    const float max_value,
    bool show = false
);

void setNormal(const cv::Mat &norm_gt_img,cv::Mat &normal_img);

cv::Mat readLabelMat(std::string FilePath);

std::map<int, vector<float>> readRatio(const string FilePath);

std::map<int, int> readType(const std::string label_type_path);

void getRealDAoLP(const cv::Mat& polar_left_img, cv::Mat& RealDoLP_img, cv::Mat& RealAoLP_img);
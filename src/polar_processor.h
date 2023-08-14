#include <fstream>
#include <sstream>
#include <iostream>

#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "PFM_ReadWrite/PFMReadWrite.h"

#include "DolpAolpToZenithAzimuth.h"

using namespace std;

// enum int
// {
// 	DIFFUSE = 0,
// 	SPECULAR = 1,
//     RAND = 2
// };

void PolarsToStokes(const vector<cv::Mat>& polars_img,vector<cv::Mat>& stokes);

void StokesToDolpAolp(const vector<cv::Mat>& stokes,cv::Mat& dolp_img,cv::Mat& aolp_img,cv::Mat& intensity_img);

void fitPolarNoise(const vector<cv::Mat>& polars_img,cv::Mat& noise_img,int type);

void PolarsToDolpAolp(const vector<cv::Mat>& polars_img,cv::Mat& dolp_img,cv::Mat& aolp_img);

void PolarsToIntensity(const vector<cv::Mat>& polars_img,cv::Mat& intensity_img);

void PolarsToS1S2(const vector<cv::Mat>& polars_img,cv::Mat& S1_img,cv::Mat& S2_img);

void PolarsToS4(const vector<cv::Mat>& polars_img,cv::Mat& S4_img);

void DolpAolpToS4(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,cv::Mat& S4_img);

void IntensityToS0(const cv::Mat& intensity_img,cv::Mat& s0);

void DolpAolpToPolars(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& polars_img);
void DolpAolpToPolars(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& polars_img,
                      const cv::Mat& dolp_img_false,const cv::Mat& aolp_img_false);

void DolpAolpToStokes(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& stokes);

void DolpAolpToNormal(const cv::Mat& dolp_img,const cv::Mat& aolp_img,int ref_type, vector<cv::Mat>& normal_img);

void DolpAolpToZenithAzimuth(const cv::Mat& dolp_img,const cv::Mat& aolp_img,int ref_type, cv::Mat& zenith_img,cv::Mat& azimuth_img);

void DolpToZenith(const cv::Mat& dolp_img,int ref_type, cv::Mat& zenith_img);

void AolpToAzimuth(const cv::Mat& aolp_img,int ref_type, cv::Mat& azimuth_img);

void StokesToPolars(const vector<cv::Mat>& stokes,vector<cv::Mat>& polars);

void StokesVisToRaw(const vector<cv::Mat>& vis_stokes,vector<cv::Mat>& stokes);

void show_ColorMap();

void visualizeDolpAolp(const cv::Mat& dolp_img,const cv::Mat& aolp_img,cv::Mat &dop_vis_img,cv::Mat &aop_vis_img);

void showDolpAolp(const cv::Mat& dolp_img,const cv::Mat& aolp_img,std::string suffix="");

void showStokes(const vector<cv::Mat>& stokes,std::string suffix="");

void showPolars(const vector<cv::Mat>& polars_img,std::string suffix="");

void showIntensity(const cv::Mat& intensity_img,std::string suffix="");

void showNormal(const cv::Mat& normal_img,std::string suffix="");

void savePfmStokes(const cv::Mat& S1_img,const cv::Mat& S2_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix = "");

void savePngStokes(const cv::Mat& S1_img,const cv::Mat& S2_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix = "");

void savePngS4(const cv::Mat& S4_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix = "");

void savePngPolars(const vector<cv::Mat>& polars,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix = "");


cv::Vec3b DAoLP_2Color(float dolpValue,float aolpValue);
cv::Vec3b AoLP_2Color(float aolpValue);
char DoLP_2Gray(float dolpValue);

cv::Vec3b DAoLP2Color2(float dolpValue,float aolpValue);
cv::Vec3b AoLPHalf2Color(float aolpValue);
cv::Vec3b DoLP2Color(float dolpValue);
cv::Vec3b Norm2Color(cv::Vec3f normValue);


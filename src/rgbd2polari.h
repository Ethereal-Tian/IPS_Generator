#include <fstream>
#include <sstream>
#include <iostream>

#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

#include <mutex>
#include <condition_variable>

#include <thread>
// #include <boost/thread/thread.hpp>
#include <termio.h>

// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "PFM_ReadWrite/PFMReadWrite.h"

using namespace std;

enum Reflection_TYPE
{
	DIFFUSE = 0,
	SPECULAR = 1,
    RAND = 2
};

class polarimetricSynthesizer{
public:
    polarimetricSynthesizer();

    void readParameters(std::string config_dir,string dataset);

    cv::Vec3b DAoLP2Color(float dolpValue,float aolpValue);

    cv::Vec3b AoLP2Color(float aolpValue);

    char DoLP2Gray(float dolpValue);

    cv::Point3d computeNorm(vector<cv::Point3d> pts);

    cv::Point3d computeNorm1(vector<cv::Point3d> pts);

    void depth2Pointcloud(const cv::Mat &rgb_img,const cv::Mat &depth_img,vector<cv::Point3d> &points,vector<cv::Point3d> &colors);

    void disparity2Pointcloud(const cv::Mat &rgb_img,const cv::Mat &disp_img,vector<cv::Point3d> &points,vector<cv::Point3d> &colors);

    void normal2Pointcloud(const cv::Mat &normal_img,vector<cv::Point3d> &norms);

    cv::Mat depth2Normal(const cv::Mat &depth_img);

    cv::Mat disparity2Normal(const cv::Mat &disp_img);

    cv::Mat NormalL2RbyDisparity(const cv::Mat& normal_img,const cv::Mat& disp_img);

    cv::Mat NormalL2RbyDisparity2(const cv::Mat& normal_img,const cv::Mat& disp_img);

    cv::Mat DoLPL2RbyDisparity(const cv::Mat& DoLP_img,const cv::Mat& disp_img);

    void normal2DAoLP(const cv::Mat &normal_img,cv::Mat& DoLP_img,cv::Mat& AoLP_img,Reflection_TYPE ref_type);

    void showColorMap();

    void showRGB(const cv::Mat &rgb_img);

    void showDepth(const cv::Mat &depth_img);

    cv::Mat showDisparity(const cv::Mat &disp_img);

    void checkDisparity(const cv::Mat &disp_img);

    cv::Mat validDisparity(const cv::Mat &disp_img);

    cv::Mat DisparityL2R(const cv::Mat& disp_img);

    void getDisparityBound(const cv::Mat &disp_img,float &min_disp,float &max_disp);

    void getDepthBound(const cv::Mat &disp_img,float &min_depth,float &max_depth);

    void showStereoMatch(const cv::Mat &disp_img,const cv::Mat &left_img,const cv::Mat &right_img);

    void showStereoOffset(const cv::Mat &disp_img,const cv::Mat &left_img,const cv::Mat &right_img,bool flag = true, std::string fix="");

    void showNormal(const cv::Mat &normal_img,std::string suffix="");

    void showNormGT(const cv::Mat &norm_gt_img);

    void setNormal(const cv::Mat &norm_gt_img,cv::Mat &normal_img);

    void compareNormGT(const cv::Mat &norm_gt_img,const cv::Mat &normal_img,bool tranformed = false);

    void showDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string suffix="");

    void savePngVisDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str);

    void savePfmDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str, std::string fix="");

    void saveExrDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str);

    void savePngDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str);

    void saveValid(const cv::Mat& valid_img,std::string syn_dir,std::string view_str, std::string fix="");

    void generateRandomReflection(const cv::Mat& DDoLP_img,const cv::Mat& DAoLP_img,
                                  const cv::Mat& SDoLP_img,const cv::Mat& SAoLP_img,
                                  cv::Mat& DoLP_img, cv::Mat& AoLP_img, cv::Mat& reflect_img);
    void generateReflectionBySemSeg(const cv::Mat& DDoLP_img,const cv::Mat& DAoLP_img,
                                    const cv::Mat& SDoLP_img,const cv::Mat& SAoLP_img,
                                    const cv::Mat& Label_img,
                                    const std::map<int, int> label_type,
                                    cv::Mat& DoLP_img, cv::Mat& AoLP_img, cv::Mat& reflect_img);

    float camera_factor;
    float camera_cx;
    float camera_cy;
    float camera_fx;
    float camera_fy;
    float camere_bl; // baseline
    float camere_bf;
    float eta;// reflective index
    Reflection_TYPE reflection_type;
};

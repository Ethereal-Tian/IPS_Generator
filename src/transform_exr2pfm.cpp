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

std::string dataset_dir;
std::string save_dir;

void Exr2Pfm(std::string data_name, std::string data_list_str){

    std::string data_dir = dataset_dir + data_name;
    std::string syn_dir = dataset_dir + "/Render" + data_name;

    std::string disp_dir =  data_dir+"/d_"+data_list_str+".exr";
    std::string disp_pfm_dir = syn_dir+"/d_"+data_list_str+".pfm";

    std::cout<<disp_dir<<std::endl;
    std::cout<<disp_pfm_dir<<std::endl;

    cv::Mat disp_img = cv::imread(disp_dir, -1);
    cv::Mat disp_pfm = cv::Mat::zeros(disp_img.rows, disp_img.cols, CV_32FC1);
    for (int v = 0; v < disp_img.rows; v++){
        for (int u = 0; u < disp_img.cols; u++) {
            float dd = disp_img.at<cv::Vec3f>(v,u)[1];
            disp_pfm.at<float>(v,u) = dd;
        }
    }
    savePFM(disp_pfm,disp_pfm_dir);
}

int main(int argc, char **argv)
{

    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    // return 0;
    if(argc != 5){
        std::cout<<"Err "<<argc<<std::endl;
        return 0;
    }


    std::string dataset_name = argv[1];
    std::string subset_name = argv[2];
    std::string data_name = "/IRS/" + dataset_name + "/" + subset_name;
    
    std::string view_s_str = argv[3];
    std::string view_e_str = argv[4];

    int view_s = std::stoi(view_s_str);
    int view_e = std::stoi(view_e_str);
    for(int i=view_s;i<=view_e;i++){
        std::string view_id = std::to_string(i);
        Exr2Pfm(data_name, view_id);
        float percent = 10000* i / (view_e-view_s+1)/100.0;
        std::cout<<dataset_name<<"-"<<subset_name<<"\t"<<percent<<"%"<<std::endl;
        // if(i>3){break;}
    }
    return 0;
}

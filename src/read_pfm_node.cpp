#include "rgbd2polari.h"

std::string dataset_dir;
std::string save_dir;

void checkPFM(std::string disp_dir){

    cv::Mat disp_img= loadPFM(disp_dir);

    std::cout<<"disparity   "<<disp_img.at<float>(2,3)<<std::endl;


    int inf_nan_num = 0;

    vector<cv::Point> vec_nan;
    vector<cv::Point> vec_inf;

    for (int v = 0; v < disp_img.rows; v++){
        for (int u = 0; u < disp_img.cols; u++) {
            float disp = disp_img.at<float>(v,u);
            if(disp>=0&&disp<disp_img.cols){
                
            }else{
                if(isnan(disp)){
                    vec_nan.push_back(cv::Point(u,v));
                }else if(isinf(disp)){
                    vec_inf.push_back(cv::Point(u,v));
                }
                if(inf_nan_num++>6){
                    continue;
                }
                std::cout<<cv::Point2d(v,u)<<disp<<std::endl;
            }
        }
    }
    std::cout<<cv::Point2d(disp_img.rows*disp_img.cols,inf_nan_num)<<cv::Point(vec_nan.size(),vec_inf.size())<<std::endl;

    cv::Mat disp_vis = cv::Mat::zeros(disp_img.size(),CV_8UC3);

    for (int v = 0; v < disp_img.rows; v++){
        for (int u = 0; u < disp_img.cols; u++) {
            float disp = disp_img.at<float>(v,u);
            int tmp_i = disp / 10.0;
            disp_vis.at<cv::Vec3b>(v,u)[0] = tmp_i;
            disp_vis.at<cv::Vec3b>(v,u)[1] = tmp_i;
            disp_vis.at<cv::Vec3b>(v,u)[2] = tmp_i;
        }
    }

    for(int i =0;i<vec_inf.size();i++){
        cv::Scalar color(0, 0, 255);
        cv::circle(disp_vis, vec_inf[i], 1.0, color, 2);
    }

    for(int i =0;i<vec_nan.size();i++){
        cv::Scalar color(0, 255, 0);
        cv::circle(disp_vis, vec_nan[i], 1.0, color, 2);
    }

    // cv::resize(disp_vis, disp_vis, cv::Size(0, 0), 0.5, 0.5); 

    cv::imshow("disp_vis",disp_vis);
    cv::waitKey();
}

int main(int argc, char **argv)
{
    checkPFM("/home/tcr/DataSets/Stereo/Scene Flow/Sampler/FlyingThings3D/disparity/0006.pfm");
    checkPFM("/home/tcr/DataSets/Stereo/Scene Flow/Sampler/FlyingThings3D/disparity/0007.pfm");
    checkPFM("/home/tcr/DataSets/Stereo/Scene Flow/Sampler/FlyingThings3D/disparity/0008.pfm");

    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Adirondack-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Backpack-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Bicycle1-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Cable-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Classroom1-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Couch-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Flowers-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Jadeplant-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Mask-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Motorcycle-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Piano-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Pipes-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Playroom-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Playtable-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Recycle-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Shelves-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Shopvac-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Sticks-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Storage-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Sword1-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Sword2-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Umbrella-perfect/disp0.pfm");
    // checkPFM("/home/tcr/Datasets/Middlebury2014/perfect/Vintage-perfect/disp0.pfm");
    return 0;
}

#include "rgbd2polari.h"
#include <algorithm>

using namespace std;

polarimetricSynthesizer::polarimetricSynthesizer(){

    eta = 1.5; // reflective index

}

void polarimetricSynthesizer::readParameters(std::string config_dir,string dataset){

    if(dataset=="middlebury"){//middlebury2021 middlebury2014 scenes2006 scenes2005 scenes2003
        std::cout<<config_dir<<std::endl;

        ifstream config_file(config_dir);
        std::string cam0_str;
        std::string cam1_str;
        std::string bl_str;
        std::string doffs_str;
        std::string cam_str;

        getline(config_file,cam0_str);
        getline(config_file,cam1_str);
        getline(config_file,doffs_str);
        getline(config_file,bl_str);

        cam_str = cam0_str;

        stringstream iss_config;
        int i1 = 0;
        int i2 = 0;
        int i3 = 0;
        double unuse;
        for(int i = 0;i<cam_str.length();i++){
            if(cam_str[i] == '['){
                i1 = i+1;
            }else if(cam_str[i] == ';' && i2==0){
                i2 = i+1;
            }else if(cam_str[i] == ';' && i2>0){
                i3 = i+1;
            }
        }
        std::string str_x = cam_str.substr(i1,i2-i1);
        std::string str_y = cam_str.substr(i2,i3-i2);

        for(int i = 0;i<bl_str.length();i++){
            if(bl_str[i] == '='){
                i1 = i+1;
            }
        }

        i2 = bl_str.length();
        std::string str_bl = bl_str.substr(i1,i2-i1);

        std::cout<<"cam_str   "<<cam_str<<std::endl;
        std::cout<<"bl_str   "<<bl_str<<std::endl;
        std::cout<<"str_x   "<<str_x<<std::endl;
        std::cout<<"str_y   "<<str_y<<std::endl;
        std::cout<<"str_bl   "<<str_bl<<std::endl;

        iss_config = stringstream(str_x);
        iss_config>>camera_fx>>unuse>>camera_cx;
        iss_config = stringstream(str_y);
        iss_config>>unuse>>camera_fy>>camera_cy;
        iss_config = stringstream(str_bl);
        iss_config>>camere_bl;
        camere_bf =  camera_fx * camere_bl / 1000.0;

        std::cout<<"camera_fx   "<<camera_fx<<std::endl;
        std::cout<<"camera_fy   "<<camera_fy<<std::endl;
        std::cout<<"camera_cx   "<<camera_cx<<std::endl;
        std::cout<<"camera_cy   "<<camera_cy<<std::endl;
        std::cout<<"camere_bl   "<<camere_bl<<std::endl;
    }

    if(dataset=="kitti2015"){//kitti2015
        camera_fx = 984.2439;
        camera_fy = 980.8141;
        camera_cx = 690.0000;
        camera_cy = 233.1966;
        camere_bl = 537;
        camere_bf=  camera_fx * camere_bl / 1000.0;
    }

    if(dataset=="FlyingThings3D"){//FlyingThings3D
        camera_fx = 984.2439;
        camera_fy = 980.8141;
        camera_cx = 690.0000;
        camera_cy = 233.1966;
        camere_bl = 1;
        camere_bf=  camera_fx * camere_bl / 1000.0;
    }

    if(dataset=="InStereo2K"){//InStereo2K
        camera_fx = 500;
        camera_fy = 500;
        camera_cx = 500;
        camera_cy = 400;
        camere_bl = 1000;
        camere_bf=  camera_fx * camere_bl / 100.0;
    }

    if(dataset=="IRS"){//IRS
        //from dataloader.SIRSLoader import SIRSDataset -IRS
        //from theIRS/Auxiliary/CameraPos/Home/XXXX/Camera.txt
        camera_fx = 480;
        camera_fy = 480;
        camera_cx = 479.5;
        camera_cy = 269.5;
        camere_bl = 1000;
        camere_bf=  camera_fx * camere_bl / 100.0;
    }

    if(dataset=="BlendedMVS"){//BlendedMVS
        camera_fx = 565.665;
        camera_fy = 565.665;
        camera_cx = 380.88 ;
        camera_cy = 287.361;
        camera_factor = 1.0;
    }

    if(dataset=="ETH3D"){//ETH3D
        camera_fx = 544.163;
        camera_fy = 543.351;
        camera_cx = 498.399 ;
        camera_cy = 279.61;
        camera_factor = 1.0;
    }

    if(dataset=="DTU"){//DTU
        camera_fx = 544.163;
        camera_fy = 543.351;
        camera_cx = 498.399 ;
        camera_cy = 279.61;
        camera_factor = 1.0;
    }

    if(dataset=="Pericipio"){
        camera_fx = 1116.615601;
        camera_fy = 1116.615601;
        camera_cx = 633.203430;
        camera_cy = 469.629089;
        camera_factor = 1000.0;
    }

    if(dataset == "SPD"){
        camera_fx = 1192.7164;
        camera_fy = 1192.7108;
        camera_cx = 664.3392;
        camera_cy = 482.9869;
        camera_factor = 1.0;
        camere_bl =  0.090886;
        camere_bf = camere_bl * camera_fx;
    }
    // camera_fx = 400;
    // camera_fy = 400;
    // camera_cx = 400;
    // camera_cy = 400;
    // camera_factor = 1.0;
    // camere_bl = 0.2;
    // camere_bf = 80;

    if(dataset=="joinMap"){//joinMap
        camera_fx = 518.0;
        camera_fy = 519.0;
        camera_cx = 325.5;
        camera_cy = 253.5;
        camera_factor = 1000.0;
    }
}

cv::Vec3b polarimetricSynthesizer::DAoLP2Color(float dolpValue,float aolpValue){
        aolpValue = aolpValue / M_PI * 180;
        dolpValue = dolpValue * 255.0;
        if(aolpValue<0){
            aolpValue += 180;
        }
        double hue = (aolpValue)*2.0;
        double saturation = dolpValue / 255.0;
        int value = 255;

        int c = round(value * saturation);
        double h = hue / 60.0;
        int x = round(c * (1 - abs(fmod(h, 2.0) - 1)));

        int blue = 0.0;
        int green  = 0.0;
        int red  = 0.0;

        if ((h >= 0) && (h <= 1)){
            blue = value - c;
            green = x + value - c;
            red = value;
        }else if ((h >= 1) && (h <= 2)){
            blue = value - c;
            green = value;
            red = x + value - c;
        }else if ((h >= 2) && (h <= 3)){
            blue = x + value - c;
            green = value;
            red = value - c;
        }else if ((h >= 3) && (h <= 4)){
            blue = value;
            green = x + value - c;
            red = value - c;
        }else if ((h >= 4) && (h <= 5)){
            blue = value;
            green = value - c;
            red = x + value - c;
        }else if ((h >= 5) && (h <= 6)){
            blue = x + value - c;
            green = value - c;
            red = value;
        }
        return cv::Vec3b(red,green,blue);
}

cv::Vec3b polarimetricSynthesizer::AoLP2Color(float aolpValue){
        aolpValue = aolpValue / M_PI * 180;
        if(aolpValue<0){
            aolpValue += 180;
        }

        double hue = (aolpValue)*2.0;
        double saturation = 1.0;
        int value = 255;

        int c = round(value * saturation);
        double h = hue / 60.0;
        int x = round(c * (1 - abs(fmod(h, 2.0) - 1)));

        int blue = 0.0;
        int green  = 0.0;
        int red  = 0.0;

        if ((h >= 0) && (h <= 1)){
            blue = value - c;
            green = x + value - c;
            red = value;
        }else if ((h >= 1) && (h <= 2)){
            blue = value - c;
            green = value;
            red = x + value - c;
        }else if ((h >= 2) && (h <= 3)){
            blue = x + value - c;
            green = value;
            red = value - c;
        }else if ((h >= 3) && (h <= 4)){
            blue = value;
            green = x + value - c;
            red = value - c;
        }else if ((h >= 4) && (h <= 5)){
            blue = value;
            green = value - c;
            red = x + value - c;
        }else if ((h >= 5) && (h <= 6)){
            blue = x + value - c;
            green = value - c;
            red = value;
        }
        return cv::Vec3b(red,green,blue);

}

char polarimetricSynthesizer::DoLP2Gray(float dolpValue){
    // int gray_tmp =  dolpValue;
    int gray_tmp = round(dolpValue * 255);
    char res = gray_tmp;
    return res;
}

cv::Point3d polarimetricSynthesizer::computeNorm(vector<cv::Point3d> pts){

        // Eigen::Vector3d P1(point_img.at<cv::Vec3f>(v,u+1)[0],point_img.at<cv::Vec3f>(v,u+1)[1],point_img.at<cv::Vec3f>(v,u+1)[2]);
        // Eigen::Vector3d P2(point_img.at<cv::Vec3f>(v,u -1)[0],point_img.at<cv::Vec3f>(v,u -1)[1],point_img.at<cv::Vec3f>(v,u -1)[2]);
        // Eigen::Vector3d P3(point_img.at<cv::Vec3f>(v+1,u)[0],point_img.at<cv::Vec3f>(v+1,u)[1],point_img.at<cv::Vec3f>(v+1,u)[2]);
        // Eigen::Vector3d P4(point_img.at<cv::Vec3f>(v -1,u)[0],point_img.at<cv::Vec3f>(v -1,u)[1],point_img.at<cv::Vec3f>(v -1,u)[2]);

        Eigen::Vector3d P1(pts[1].x,pts[1].y,pts[1].z); //(v,u+1)
        Eigen::Vector3d P2(pts[2].x,pts[2].y,pts[2].z); //(v,u -1)
        Eigen::Vector3d P3(pts[3].x,pts[3].y,pts[3].z); //(v+1,u)
        Eigen::Vector3d P4(pts[4].x,pts[4].y,pts[4].z); //(v -1,u)
        Eigen::Vector3d LX = P1 - P2;
        Eigen::Vector3d LY = P3 - P4;

        Eigen::Vector3d NN = LX.cross(LY);
        return cv::Point3d(NN.x(),NN.y(),NN.z());
}

cv::Point3d polarimetricSynthesizer::computeNorm1(vector<cv::Point3d> pts){

        // float dx_dz =  (point_img.at<cv::Vec3f>(v,u+1)[0] - point_img.at<cv::Vec3f>(v,u-1)[0])
        //                                 /(point_img.at<cv::Vec3f>(v,u+1)[2] - point_img.at<cv::Vec3f>(v,u-1)[2]);
        // float dy_dz =  (point_img.at<cv::Vec3f>(v+1,u)[1] - point_img.at<cv::Vec3f>(v-1,u)[1])
        //                                 /(point_img.at<cv::Vec3f>(v+1,u)[2] - point_img.at<cv::Vec3f>(v-1,u)[2]);

        float dx_dz = (pts[1].x - pts[2].x) / (pts[1].z - pts[2].z);
        float dy_dz = (pts[3].y - pts[4].y) / (pts[3].z - pts[4].z);

        Eigen::Vector3d NN(dx_dz, dy_dz, 1.0);
        return cv::Point3d(NN.x(),NN.y(),NN.z());
}

void polarimetricSynthesizer::depth2Pointcloud(const cv::Mat &rgb_img,const cv::Mat &depth_img,vector<cv::Point3d> &points,vector<cv::Point3d> &colors){
    // std::cout<<"Depth To Pointcloud"<<std::endl;
    points.clear();
    colors.clear();
    if(depth_img.type() == CV_16UC1){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
                int dd = depth_img.at<ushort>(v,u);
                if (dd < 0.01) {
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = 1.0 * dd / camera_factor;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }else if(depth_img.type() == CV_32FC1){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
                float dd = depth_img.at<float>(v,u);
                if (dd < 0.0000001){
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = 1.0 * dd / camera_factor;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }
    return;
}

void polarimetricSynthesizer::disparity2Pointcloud(const cv::Mat &rgb_img,const cv::Mat &disp_img,vector<cv::Point3d> &points,vector<cv::Point3d> &colors){
    // std::cout<<"Disparity To Pointcloud"<<std::endl;
    points.clear();
    colors.clear();
    if(disp_img.type() == CV_32FC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<float>(v,u);
                if (dd < 0.01) {
                    // points.push_back(cv::Point3d(0,0,0));
                    // colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = camere_bf / dd;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }else if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<cv::Vec3f>(v,u)[1];
                if (dd < 0.01) {
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = camere_bf / dd;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }else if(disp_img.type() == CV_16UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<ushort>(v,u);
                dd /= 100.0;
                if (dd < 0.01) {
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = camere_bf / dd;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }else if(disp_img.type() == CV_8UC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                int dd = disp_img.at<cv::Vec3b>(v,u)[0];
                if (dd < 0.01) {
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = 1.0 * camere_bf / dd;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }else if(disp_img.type() == CV_8UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                int dd = disp_img.at<uchar>(v,u);
                if (dd < 0.01) {
                    points.push_back(cv::Point3d(0,0,0));
                    colors.push_back(cv::Point3d(0,0,0));
                    continue;
                }
                cv::Point3d point;
                point.z = 1.0 * camere_bf / dd;
                point.x = (u - camera_cx) / camera_fx * point.z;
                point.y = (v - camera_cy) / camera_fy * point.z;
                cv::Point3d color;
                color.x = rgb_img.at<cv::Vec3b>(v,u)[0];
                color.y = rgb_img.at<cv::Vec3b>(v,u)[1];
                color.z = rgb_img.at<cv::Vec3b>(v,u)[2];
                points.push_back(point);
                colors.push_back(color);
            }
        }
    }
    return;
}

void polarimetricSynthesizer::normal2Pointcloud(const cv::Mat &normal_img,vector<cv::Point3d> &norms){
    norms.clear();
    for (int v = 0; v < normal_img.rows; v++){
        for (int u = 0; u < normal_img.cols; u++) {
            if(normal_img.at<cv::Vec3f>(v,u)[0] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[1] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[2] == 0){
                norms.push_back(cv::Point3d(0,0,0));
                continue;
            }

            float nx = normal_img.at<cv::Vec3f>(v,u)[0];
            float ny = normal_img.at<cv::Vec3f>(v,u)[1];
            float nz = normal_img.at<cv::Vec3f>(v,u)[2];
            norms.push_back(cv::Point3d(nx,ny,nz));
        }
    }
}

cv::Mat polarimetricSynthesizer::depth2Normal(const cv::Mat &depth_img){
    // std::cout<<"Depth To Normal"<<std::endl;

    cv::Mat point_img = cv::Mat::zeros(depth_img.rows,depth_img.cols,CV_32FC3);
    if(depth_img.type() == CV_8UC3){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
               int dd = depth_img.at<cv::Vec3b>(v,u)[0];
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = 1.0 * dd / camera_factor;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(depth_img.type() == CV_16UC1){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
               int dd = depth_img.at<ushort>(v,u);
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = 1.0 * dd / camera_factor;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(depth_img.type() == CV_32FC1){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
                float dd = depth_img.at<float>(v,u);
                if (dd < 0.0000001) continue; // 为0表示没有测量到
                float zz = 1.0 * dd / camera_factor;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }

    cv::Mat normal_img = cv::Mat::zeros(depth_img.rows,depth_img.cols,CV_32FC3);
    for (int v = 1; v < depth_img.rows - 1; v++){
        for (int u = 1; u < depth_img.cols - 1; u++) {
            float z1 = point_img.at<cv::Vec3f>(v+1,u)[2];
            float z2 = point_img.at<cv::Vec3f>(v-1,u)[2];
            float z3 = point_img.at<cv::Vec3f>(v,u+1)[2];
            float z4 = point_img.at<cv::Vec3f>(v,u-1)[2];
            if(z1<0.01||
                z2<0.01||
                z3<0.01||
                z4<0.01){
                continue;
            }
            Eigen::Vector3d P1(point_img.at<cv::Vec3f>(v,u+1)[0],point_img.at<cv::Vec3f>(v,u+1)[1],point_img.at<cv::Vec3f>(v,u+1)[2]);
            Eigen::Vector3d P2(point_img.at<cv::Vec3f>(v,u-1)[0],point_img.at<cv::Vec3f>(v,u -1)[1],point_img.at<cv::Vec3f>(v,u -1)[2]);
            Eigen::Vector3d P3(point_img.at<cv::Vec3f>(v+1,u)[0],point_img.at<cv::Vec3f>(v+1,u)[1],point_img.at<cv::Vec3f>(v+1,u)[2]);
            Eigen::Vector3d P4(point_img.at<cv::Vec3f>(v -1,u)[0],point_img.at<cv::Vec3f>(v -1,u)[1],point_img.at<cv::Vec3f>(v -1,u)[2]);

            Eigen::Vector3d LX = P1 - P2;
            Eigen::Vector3d LY = P3 - P4;

            Eigen::Vector3d NN = LX.cross(LY);
            NN.normalize();
            if(NN.z()>0){
                NN = -NN;
            }
            normal_img.at<cv::Vec3f>(v,u) = cv::Vec3f(NN.x(), NN.y(), NN.z());
        }
    }

    return normal_img;
}

cv::Mat polarimetricSynthesizer::disparity2Normal(const cv::Mat &disp_img){
    // std::cout<<"Disparity To Normal"<<std::endl;

    cv::Mat point_img = cv::Mat::zeros(disp_img.rows,disp_img.cols,CV_32FC3);
    // cout << "camere_bf: " << camere_bf << endl;
    // cout << "camera_cx: " << camera_cx << endl;
    // cout << "camera_cy: " << camera_cy << endl;

    if(disp_img.type() == CV_32FC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<float>(v,u);
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = camere_bf / dd;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<cv::Vec3f>(v,u)[1];
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = camere_bf / dd;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(disp_img.type() == CV_16UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<ushort>(v,u);
                dd /= 100.0;
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = camere_bf / dd;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(disp_img.type() == CV_8UC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                int dd = disp_img.at<cv::Vec3b>(v,u)[0];
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = 1.0 * camere_bf / dd;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else if(disp_img.type() == CV_8UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                int dd = disp_img.at<uchar>(v,u);
                if (dd < 0.01) continue; // 为0表示没有测量到
                float zz = 1.0 * camere_bf / dd;
                float xx = (u - camera_cx) / camera_fx * zz;
                float yy = (v - camera_cy) / camera_fy * zz;
                point_img.at<cv::Vec3f>(v,u)[0] = xx;
                point_img.at<cv::Vec3f>(v,u)[1] = yy;
                point_img.at<cv::Vec3f>(v,u)[2] = zz;
            }
        }
    }else{
        cout << disp_img.type() << " not defined" << endl;
        system("pause");
    }


    cv::Mat normal_img = cv::Mat::zeros(disp_img.rows,disp_img.cols,CV_32FC3);
    for (int v = 1; v < disp_img.rows - 1; v++){
        for (int u = 1; u < disp_img.cols - 1; u++) {
            float z1 = point_img.at<cv::Vec3f>(v+1,u)[2];
            float z2 = point_img.at<cv::Vec3f>(v-1,u)[2];
            float z3 = point_img.at<cv::Vec3f>(v,u+1)[2];
            float z4 = point_img.at<cv::Vec3f>(v,u-1)[2];
            if(z1<0.01||
                z2<0.01||
                z3<0.01||
                z4<0.01){
                continue;
            }
            Eigen::Vector3d P1(point_img.at<cv::Vec3f>(v,u+1)[0],point_img.at<cv::Vec3f>(v,u+1)[1],point_img.at<cv::Vec3f>(v,u+1)[2]);
            Eigen::Vector3d P2(point_img.at<cv::Vec3f>(v,u -1)[0],point_img.at<cv::Vec3f>(v,u -1)[1],point_img.at<cv::Vec3f>(v,u -1)[2]);
            Eigen::Vector3d P3(point_img.at<cv::Vec3f>(v+1,u)[0],point_img.at<cv::Vec3f>(v+1,u)[1],point_img.at<cv::Vec3f>(v+1,u)[2]);
            Eigen::Vector3d P4(point_img.at<cv::Vec3f>(v -1,u)[0],point_img.at<cv::Vec3f>(v -1,u)[1],point_img.at<cv::Vec3f>(v -1,u)[2]);

            Eigen::Vector3d LX = P1 - P2;
            Eigen::Vector3d LY = P3 - P4;

            Eigen::Vector3d NN = LX.cross(LY);
            NN.normalize();
            if(NN.z()>0){
                NN = -NN;
            }
            normal_img.at<cv::Vec3f>(v,u) = cv::Vec3f(NN.x(), NN.y(), NN.z());
        }
    }
    return normal_img;
}

bool ucmp(std::pair<float,int> &a, std::pair<float,int> &b){
    return a.first<b.first;
}

bool ucmp2(std::pair<float, float> a, std::pair<float, float> b){
    return a.first<b.first;
}

bool ucmp3(std::pair<int,int> &a, std::pair<int,int> &b){
    if (a.first != b.first)
        return a.first<b.first;
    else return a.second < b.second;
}

cv::Mat polarimetricSynthesizer::DisparityL2R(const cv::Mat& disp_img){
    cv::Mat dispR_img = cv::Mat::zeros(disp_img.rows,disp_img.cols,disp_img.type());
    if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            std::vector<std::pair<float,int>> pairs_tmp;
            for (int u = 0; u < disp_img.cols; u++) {
                pairs_tmp.push_back(make_pair(u,-1));
                float dd = disp_img.at<cv::Vec3f>(v,u)[1];
                if (dd < 0.01) continue; // 为0表示没有测量到
                float u2 = 1.0*u - dd;
                pairs_tmp.push_back(make_pair(u2,u));
            }
            std::sort(pairs_tmp.begin(), pairs_tmp.end(),ucmp);

            for(int i = 1;i<pairs_tmp.size()-1;i++){
                if(pairs_tmp[i].second == -1){
                    int u1  = pairs_tmp[i-1].second;
                    int u2  = pairs_tmp[i+1].second;
                    float x1  = pairs_tmp[i-1].first;
                    float x2  = pairs_tmp[i+1].first;
                    float x0 = pairs_tmp[i].first;
                    int u0  = round(x0);

                    if(u1 == -1||u2 == -1){
                        continue;
                    }
                    if((x0 - x1)> 1||(x2 - x0)>1){
                        continue;
                    }
                    float disp1 = disp_img.at<cv::Vec3f>(v,u1)[1];
                    float disp2 = disp_img.at<cv::Vec3f>(v,u2)[1];
                    float disp0 =  ((x0 - x1) * disp2 + (x2 - x0) * disp1)/(x2 - x1);
                    dispR_img.at<cv::Vec3f>(v,u0)[1] = disp0;
                }
            }
        }
    }
    return dispR_img;
}

cv::Mat polarimetricSynthesizer::NormalL2RbyDisparity2(const cv::Mat& normal_img,const cv::Mat& disp_img){
    cv::Mat normalR_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC3);
    cv::Mat visit = cv::Mat::zeros(normal_img.rows, normal_img.cols, CV_8UC1);

    for (int v = 0; v < normal_img.rows; v++){
        std::vector<std::pair<float,int>> pairs_tmp;
        for (int col_L = 0; col_L < normal_img.cols; col_L++) {
            //pairs_tmp.push_back(make_pair(col_L,-1));
            float dd;
            if (disp_img.type() == CV_32FC3)
                dd = disp_img.at<cv::Vec3f>(v,col_L)[1];
            else if (disp_img.type() == CV_32FC1)
                dd = disp_img.at<float>(v,col_L);
            if (dd < 0.01) continue; // 为0表示没有测量到
            float col_R = 1.0*col_L - dd;
            pairs_tmp.push_back(make_pair(col_R, col_L));
        }
        if(pairs_tmp.size() == 0) continue;
        std::sort(pairs_tmp.begin(), pairs_tmp.end(),ucmp);


        std::vector<int> low_index(normal_img.cols, -1);
        std::vector<int> high_index(normal_img.cols, -1);
        // cout << normal_img.cols << " " << pairs_tmp.size() << endl;
        for(int col = 0; col < normal_img.cols; col++){
            int i = 0;
            while(pairs_tmp[i].first <= col && i < pairs_tmp.size()) i++;
            low_index[col] = max(i-1, 0);
            int j = pairs_tmp.size()-1;
            while(pairs_tmp[j].first >= col && j >= 0) j--;
            high_index[col] = min(j+1, int(pairs_tmp.size())-1);
        }
        const int max_dist = 4;

        for(int i = 0; i < normal_img.cols; i++){
            float col_R_low = pairs_tmp[low_index[i]].first;
            float col_R_high = pairs_tmp[high_index[i]].first;
            int col_R = i;
            int col_L_low = pairs_tmp[low_index[i]].second;
            int col_L_high = pairs_tmp[high_index[i]].second;
            if( col_R_high - col_R_low > max_dist) continue;

            if (col_L_low < 0 || col_L_high >= normal_img.cols){
                cout << i << endl;
                cout << pairs_tmp.size() << " " << low_index[i] << " " << high_index[i] << endl;
                cout << col_R_low << " " << col_R_high << endl;
                cout << col_L_low << " " << col_L_high << endl;
                system("pause");
            }
            Eigen::Vector3d N1(normal_img.at<cv::Vec3f>(v,col_L_low)[0],
                                normal_img.at<cv::Vec3f>(v,col_L_low)[1],
                                normal_img.at<cv::Vec3f>(v,col_L_low)[2]);

            Eigen::Vector3d N2(normal_img.at<cv::Vec3f>(v,col_L_high)[0],
                                normal_img.at<cv::Vec3f>(v,col_L_high)[1],
                                normal_img.at<cv::Vec3f>(v,col_L_high)[2]);

            Eigen::Vector3d N0 =  ((col_R - col_R_low) * N2 + (col_R_high - col_R) * N1)/(col_R_high - col_R_low);
            // if(v == 0) cout << N1.transpose() << " " << N2.transpose() << " " << N0.transpose() << " " << endl;
            N0.normalize();
            normalR_img.at<cv::Vec3f>(v,col_R)[0] = N0(0);
            normalR_img.at<cv::Vec3f>(v,col_R)[1] = N0(1);
            normalR_img.at<cv::Vec3f>(v,col_R)[2] = N0(2);
        }

    }

    return normalR_img;
}

cv::Mat polarimetricSynthesizer::NormalL2RbyDisparity(const cv::Mat& normal_img,const cv::Mat& disp_img){
    cv::Mat normalR_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC3);
    cv::Mat visit = cv::Mat::zeros(normal_img.rows, normal_img.cols, CV_8UC1);
    for (int v = 0; v < disp_img.rows; v++){
        std::vector<std::pair<float,int>> pairs_tmp;
        for (int col_L = 0; col_L < disp_img.cols; col_L++) {
            pairs_tmp.push_back(make_pair(col_L,-1));
            float dd;
            if (disp_img.type() == CV_32FC3)
                dd = disp_img.at<cv::Vec3f>(v,col_L)[1];
            else if (disp_img.type() == CV_32FC1)
                dd = disp_img.at<float>(v,col_L);
            if (dd < 0.01) continue; // 为0表示没有测量到
            float col_R = 1.0*col_L - dd;
            pairs_tmp.push_back(make_pair(col_R, col_L));
        }
        std::sort(pairs_tmp.begin(), pairs_tmp.end(),ucmp);


        for(int i = 1;i<pairs_tmp.size()-1;i++){
            if(pairs_tmp[i].second == -1){
                int col_L_low  = pairs_tmp[i-1].second; // u1
                int col_L_high  = pairs_tmp[i+1].second;// u2
                float col_R_low  = pairs_tmp[i-1].first;       // x1
                float col_R_high  = pairs_tmp[i+1].first;       // x2
                float col_R = pairs_tmp[i].first;

                if(col_L_low == -1 || col_L_high == -1){
                    continue;
                }

                if((col_R - col_R_low)> 1||(col_R_high - col_R)>1){
                    continue;
                }

                Eigen::Vector3d N1(normal_img.at<cv::Vec3f>(v,col_L_low)[0],
                                    normal_img.at<cv::Vec3f>(v,col_L_low)[1],
                                    normal_img.at<cv::Vec3f>(v,col_L_low)[2]);
                Eigen::Vector3d N2(normal_img.at<cv::Vec3f>(v,col_L_high)[0],
                                    normal_img.at<cv::Vec3f>(v,col_L_high)[1],
                                    normal_img.at<cv::Vec3f>(v,col_L_high)[2]);
                Eigen::Vector3d N0 =  ((col_R - col_R_low) * N2 + (col_R_high - col_R) * N1)/(col_R_high - col_R_low);
                if(v == 100) cout << N1.transpose() << " " << N2.transpose() << " " << N0.transpose() << " " << endl;
                N0.normalize();
                normalR_img.at<cv::Vec3f>(v,round(col_R))[0] = N0(0);
                normalR_img.at<cv::Vec3f>(v,round(col_R))[1] = N0(1);
                normalR_img.at<cv::Vec3f>(v,round(col_R))[2] = N0(2);
                if(v == 100) cout << N1.transpose() << " " << N2.transpose() << " " << N0.transpose() << " " << endl;
                if(v == 100) cout << endl;
            }
        }
    }

    return normalR_img;
}

void polarimetricSynthesizer::normal2DAoLP(
    const cv::Mat &normal_img,
    cv::Mat& DoLP_img, cv::Mat& AoLP_img, Reflection_TYPE ref_type){
    reflection_type = ref_type;
    // std::cout<<"Normal To DoLP and AoLP"<<std::endl;

// nx = cos(azimuth)sin(zenith)
// ny = -sin(azimuth)sin(zenith)
// nz = -cos(zenith)
{
    // std::cout<<"atan2(1,1)"<<atan2(1,1)<<std::endl;
    // std::cout<<"atan2(-1,1)"<<atan2(-1,1)<<std::endl;
    // std::cout<<"atan2(1,-1)"<<atan2(1,-1)<<std::endl;
    // std::cout<<"atan2(-1,-1)"<<atan2(-1,-1)<<std::endl;
    // std::cout<<"atan2(0,1)"<<atan2(0.1,1)<<std::endl;
    // std::cout<<"atan2(0,-1)"<<atan2(0.1,-1)<<std::endl;
    // std::cout<<"atan2(0,1)"<<atan2(-0.1,1)<<std::endl;
    // std::cout<<"atan2(0,-1)"<<atan2(-0.1,-1)<<std::endl;
    // std::cout<<"acos(-1)"<<acos(-1)<<std::endl;
    // std::cout<<"acos(1)"<<acos(1)<<std::endl;
    // std::cout<<"acos(0.1)"<<acos(0.1)<<std::endl;
    // std::cout<<"acos(-0.1)"<<acos(-0.1)<<std::endl;
}
{
    // for(int i = 0;i<=6;i++){
    //     float specular_rho,diffuse_rho,zenith;
    //     zenith = i/6.0 * M_PI;
    //     specular_rho =  2*sin(zenith)*sin(zenith)*cos(zenith) * sqrt(eta*eta-sin(zenith)*sin(zenith))/
    //                                     (eta*eta - sin(zenith)*sin(zenith) - eta*eta * sin(zenith)*sin(zenith) + 2*sin(zenith)*sin(zenith)*sin(zenith)*sin(zenith));

    //     diffuse_rho =  (pow((eta-1/eta),2)*sin(zenith)*sin(zenith))/
    //                                         (2+2*eta*eta-pow((eta+1/eta),2)*sin(zenith)*sin(zenith) + 4*cos(zenith)*sqrt(eta*eta-sin(zenith)*sin(zenith)));
    //     std::cout<<cv::Point3d(zenith,specular_rho,diffuse_rho)<<std::endl;
    // }
}

    // double max_tmp = -10.0;
    // double min_tmp = 10.0;
    // double mean_tmp = 0.0;


    cv::Mat azimuth_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    cv::Mat zenith_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);

    for (int v = 1; v < normal_img.rows - 1; v++){
        for (int u = 1; u < normal_img.cols - 1; u++) {
            if(normal_img.at<cv::Vec3f>(v,u)[0] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[1] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[2] == 0){
                continue;
            }

            float nx = normal_img.at<cv::Vec3f>(v,u)[0];
            float ny = normal_img.at<cv::Vec3f>(v,u)[1];
            float nz = normal_img.at<cv::Vec3f>(v,u)[2];

            float zenith = acos(-nz);
            // float azimuth = atan(-ny/nx);
            float azimuth = atan2(-ny,nx);

            azimuth_img.at<float>(v,u) = azimuth;
            zenith_img.at<float>(v,u) = zenith;

            // double val_tmp = azimuth;
            // double val_tmp = zenith;
            // double val_tmp = zenith;
            // double val_tmp = nz;
            // max_tmp = max_tmp>val_tmp?max_tmp:val_tmp;
            // min_tmp = min_tmp<val_tmp?min_tmp:val_tmp;
            // mean_tmp += val_tmp;
        }
    }

    cv::Mat diffuse_rho_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    cv::Mat diffuse_phi_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    cv::Mat specular_rho_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    cv::Mat specular_phi_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);



    for (int v = 1; v < normal_img.rows - 1; v++){
        for (int u = 1; u < normal_img.cols - 1; u++) {
            if(normal_img.at<cv::Vec3f>(v,u)[0] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[1] == 0 &&
                normal_img.at<cv::Vec3f>(v,u)[2] == 0){
                continue;
            }

            float zenith = zenith_img.at<float>(v,u);
            float azimuth = azimuth_img.at<float>(v,u);

            float specular_rho =  2*sin(zenith)*sin(zenith)*cos(zenith) * sqrt(eta*eta-sin(zenith)*sin(zenith))/
                                                        (eta*eta - sin(zenith)*sin(zenith) - eta*eta * sin(zenith)*sin(zenith) + 2*sin(zenith)*sin(zenith)*sin(zenith)*sin(zenith));

            float diffuse_rho =  (pow((eta-1/eta),2)*sin(zenith)*sin(zenith))/
                                                            (2+2*eta*eta-pow((eta+1/eta),2)*sin(zenith)*sin(zenith) + 4*cos(zenith)*sqrt(eta*eta-sin(zenith)*sin(zenith)));

            float diffuse_phi;
            float specular_phi;
            if(0){// (-pi, pi)
                if(1){// azimuth;
                    diffuse_phi = azimuth;
                }else{// azimuth ± 180
                    if(azimuth < 0.0){
                        diffuse_phi = azimuth + M_PI;
                    }else{
                        diffuse_phi = azimuth - M_PI;
                    }
                }

                if(1){// azimuth - 90;
                    if(azimuth < -M_PI / 2.0){
                        specular_phi = azimuth + M_PI * 3.0 / 2.0;
                    }else{
                        specular_phi = azimuth - M_PI / 2.0;
                    }
                }else{// azimuth + 90;
                    if(azimuth > M_PI / 2.0){
                        specular_phi = azimuth - M_PI * 3.0 / 2.0;
                    }else{
                        specular_phi = azimuth + M_PI / 2.0;
                    }
                }
            }

            if(1){// (0 - pi)
                if(azimuth >0 ){
                    diffuse_phi = azimuth;
                }else{
                    diffuse_phi = azimuth + M_PI;
                }

                if(azimuth >= M_PI / 2.0){
                    specular_phi = azimuth - M_PI / 2.0;
                }else if(azimuth >= -M_PI / 2.0){
                    specular_phi = azimuth + M_PI / 2.0;
                }else{
                    specular_phi = azimuth + M_PI * 3.0 / 2.0;
                }
            }

            // double val_tmp = azimuth;
            // double val_tmp = zenith;

            // double val_tmp = diffuse_phi;
            // double val_tmp = specular_phi;
            // double val_tmp = diffuse_rho;
            // double val_tmp = specular_rho;

            // max_tmp = max_tmp>val_tmp?max_tmp:val_tmp;
            // min_tmp = min_tmp<val_tmp?min_tmp:val_tmp;

            diffuse_rho_img.at<float>(v,u) = diffuse_rho;
            diffuse_phi_img.at<float>(v,u) = diffuse_phi;
            specular_rho_img.at<float>(v,u) = specular_rho;
            specular_phi_img.at<float>(v,u) = specular_phi;
        }
    }

    // std::cout<<"############# max_tmp    "<<max_tmp<<std::endl;
    // std::cout<<"############# min_tmp    "<<min_tmp<<std::endl;
    // std::cout<<"############# mean_tmp    "<<mean_tmp/ normal_img.rows / normal_img.cols<<std::endl;

    if(reflection_type == 	DIFFUSE){
        DoLP_img = diffuse_rho_img.clone();
        AoLP_img = diffuse_phi_img.clone();
    }else if(reflection_type == SPECULAR){
        DoLP_img = specular_rho_img.clone();
        AoLP_img = specular_phi_img.clone();
    }
}


void polarimetricSynthesizer::generateRandomReflection(
    const cv::Mat& DDoLP_img,const cv::Mat& DAoLP_img,
    const cv::Mat& SDoLP_img,const cv::Mat& SAoLP_img,
    cv::Mat& DoLP_img, cv::Mat& AoLP_img, cv::Mat& reflect_img){

    std::cout<<"rand num "<<rand()<<std::endl;
    DoLP_img = cv::Mat::zeros(DDoLP_img.rows,DDoLP_img.cols,CV_32FC1);
    AoLP_img = cv::Mat::zeros(DAoLP_img.rows,DAoLP_img.cols,CV_32FC1);
    reflect_img = cv::Mat::zeros(DDoLP_img.rows,DDoLP_img.cols,CV_8UC1);

    for (int v = 0; v < DDoLP_img.rows; v++){
        for (int u = 0; u < DDoLP_img.cols; u++) {
            double rand_num = round( ((double)rand()) / ((double)RAND_MAX) );
            if(rand_num>0.1){
                DoLP_img.at<float>(v,u) = DDoLP_img.at<float>(v,u);
                AoLP_img.at<float>(v,u) = DAoLP_img.at<float>(v,u);
                reflect_img.at<uchar>(v,u) = 255;
            }else{
                DoLP_img.at<float>(v,u) = SDoLP_img.at<float>(v,u);
                AoLP_img.at<float>(v,u) = SAoLP_img.at<float>(v,u);
                reflect_img.at<uchar>(v,u) = 0;
            }
        }
    }

}



void polarimetricSynthesizer::showRGB(const cv::Mat &rgb_img){
    std::cout<<"rgb_img size : "<<rgb_img.size()<<std::endl;
    std::cout<<"rgb_img channels : "<<rgb_img.channels()<<std::endl;
    std::cout<<"rgb_img type : "<<rgb_img.type()<<std::endl;
    cv::imshow("rgb_img",rgb_img);
    cv::waitKey();
}

void polarimetricSynthesizer::showDepth(const cv::Mat &depth_img){
    std::cout<<"depth_img size : "<<depth_img.size()<<std::endl;
    std::cout<<"depth_img channels : "<<depth_img.channels()<<std::endl;
    std::cout<<"depth_img type : "<<depth_img.type()<<std::endl;
    std::cout<<"CV_16UC1 : "<<CV_16UC1<<std::endl;
    std::cout<<"CV_32FC1 : "<<CV_32FC1<<std::endl;
    std::cout<<"CV_8UC3 : "<<CV_8UC3<<std::endl;
    std::cout<<"CV_32FC3 : "<<CV_32FC3<<std::endl;

    float min_depth;
    float max_depth;

    getDepthBound(depth_img,min_depth,max_depth);

    cv::imshow("depth_img",depth_img);
    cv::waitKey();
}

cv::Mat polarimetricSynthesizer::showDisparity(const cv::Mat &disp_img){

    std::cout<<"disp_img size : "<<disp_img.size()<<std::endl;
    std::cout<<"disp_img channels : "<<disp_img.channels()<<std::endl;
    std::cout<<"disp_img type : "<<disp_img.type()<<std::endl;
    std::cout<<"CV_8UC1 : "<<CV_8UC1<<std::endl;
    std::cout<<"CV_16UC1 : "<<CV_16UC1<<std::endl;
    std::cout<<"CV_32FC1 : "<<CV_32FC1<<std::endl;
    std::cout<<"CV_8UC3 : "<<CV_8UC3<<std::endl;
    std::cout<<"CV_32FC3 : "<<CV_32FC3<<std::endl;

    float min_disp;
    float max_disp;

    getDisparityBound(disp_img,min_disp,max_disp);
    cv::Mat disp_vis_img = cv::Mat::zeros(disp_img.rows,disp_img.cols,CV_8UC1);
    if(disp_img.type() == CV_16UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<ushort>(v,u);
                int disp_int = round(dd /  max_disp * 255);
                disp_vis_img.at<uchar>(v,u) = disp_int;
                // std::cout<<disp_int<<" ";
            }
        }
    }else if(disp_img.type() == CV_32FC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<float>(v,u);
                int disp_int = round(dd /  max_disp * 255);
                disp_vis_img.at<uchar>(v,u) = disp_int;
                // std::cout<<disp_int<<" ";
            }
        }
    }else if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<cv::Vec3f>(v,u)[1];
                int disp_int = round(dd /  max_disp * 255);
                disp_vis_img.at<uchar>(v,u) = disp_int;
                // std::cout<<disp_int<<" ";
            }
        }
    }else if(disp_img.type() == CV_8UC3){
        disp_vis_img = disp_img.clone();
    }else if(disp_img.type() == CV_8UC1){
        disp_vis_img = disp_img.clone();
    }
    cv::imshow("disp_vis_img",disp_vis_img);
    cv::waitKey();
    return disp_vis_img;
}

void polarimetricSynthesizer::getDisparityBound(const cv::Mat &disp_img,float &min_disp,float &max_disp){
    min_disp = 999999;
    max_disp = 0;
    vector<float> min_disps(3,min_disp);
    vector<float> max_disps(3,max_disp);
    if(disp_img.type() == CV_16UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<ushort>(v,u);
                if(dd<min_disp){
                    min_disp = dd;
                }
                if (dd> max_disp){
                    max_disp = dd;
                }
            }
        }
        std::cout<<"disparty range: "<<cv::Point2d(min_disp,max_disp)<<std::endl;
    }else if(disp_img.type() == CV_32FC1){
            for (int v = 0; v < disp_img.rows; v++){
                for (int u = 0; u < disp_img.cols; u++) {
                    float dd = disp_img.at<float>(v,u);
                    if(dd<min_disp){
                        min_disp = dd;
                    }
                    if (dd> max_disp){
                        max_disp = dd;
                    }
                }
            }
            std::cout<<"disparty range: "<<cv::Point2d(min_disp,max_disp)<<std::endl;
     }else if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                cv::Vec3f dd = disp_img.at<cv::Vec3f>(v,u);
                for(int k=0;k<3;k++){
                    if(dd[k]<min_disps[k]){
                        min_disps[k] = dd[k];
                    }
                    if (dd[k]> max_disps[k]){
                        max_disps[k] = dd[k];
                    }
                }
            }
        }
        std::cout<<"disparty range: " <<cv::Point2d(min_disps[0],max_disps[0])
                                                                    <<cv::Point2d(min_disps[1],max_disps[1])
                                                                    <<cv::Point2d(min_disps[2],max_disps[2])<<std::endl;
        min_disp = min_disps[1];
        max_disp = max_disps[1];
     }else if(disp_img.type() == CV_8UC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                cv::Vec3b dd = disp_img.at<cv::Vec3b>(v,u);

                for(int k=0;k<3;k++){
                    if(dd[k]<min_disps[k]){
                        min_disps[k] = dd[k];
                    }
                    if (dd[k]> max_disps[k]){
                        max_disps[k] = dd[k];
                    }
                }
            }
        }
        std::cout<<"disparty range: " <<cv::Point2d(min_disps[0],max_disps[0])
                                                                    <<cv::Point2d(min_disps[1],max_disps[1])
                                                                    <<cv::Point2d(min_disps[2],max_disps[2])<<std::endl;
        min_disp = min_disps[1];
        max_disp = max_disps[1];
    }else if(disp_img.type() == CV_8UC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float dd = disp_img.at<uchar>(v,u);
                if(dd<min_disp){
                    min_disp = dd;
                }
                if (dd> max_disp){
                    max_disp = dd;
                }
            }
        }
        std::cout<<"disparty range: "<<cv::Point2d(min_disp,max_disp)<<std::endl;
    }
}

void polarimetricSynthesizer::getDepthBound(const cv::Mat &depth_img,float &min_depth,float &max_depth){
    min_depth = 999999.0;
    max_depth = 0.0;
    std::cout<<"min_depth "<<min_depth<<std::endl;
    std::cout<<"max_depth "<<max_depth<<std::endl;
    std::cout<<"# depth range: "<<cv::Point2d(min_depth,max_depth)<<std::endl;

    if(depth_img.type() == CV_16UC1){
        for (int v = 0; v < depth_img.rows; v++){
            for (int u = 0; u < depth_img.cols; u++) {
                float dd = depth_img.at<ushort>(v,u);
                if(dd<min_depth){
                    min_depth = dd;
                }
                if (dd> max_depth){
                    max_depth = dd;
                }
            }
        }
        std::cout<<"depth range: "<<cv::Point2d(min_depth,max_depth)<<std::endl;
    }else if(depth_img.type() == CV_32FC1){
            for (int v = 0; v < depth_img.rows; v++){
                for (int u = 0; u < depth_img.cols; u++) {
                    float dd = depth_img.at<float>(v,u);
                    if(dd<min_depth){
                        min_depth = dd;
                    }
                    if (dd> max_depth){
                        max_depth = dd;
                    }
                }
            }
            std::cout<<"depth range: "<<cv::Point2d(min_depth,max_depth)<<std::endl;
     }
}

void polarimetricSynthesizer::checkDisparity(const cv::Mat &disp_img){
    cv::Mat disp_vis_img = showDisparity(disp_img);

    int inf_nan_num = 0;
    vector<cv::Point> vec_nan;
    vector<cv::Point> vec_inf;
    vector<cv::Point> vec_zero;
    if(disp_img.type() == CV_32FC1){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float disp = disp_img.at<float>(v,u);
                if(isnan(disp)){
                    vec_nan.push_back(cv::Point(u,v));
                    inf_nan_num++;
                }else if(isinf(disp)){
                    vec_inf.push_back(cv::Point(u,v));
                    inf_nan_num++;
                }else if(disp == 0){
                    vec_zero.push_back(cv::Point(u,v));
                    continue;
                }else{
                    continue;
                }
                if(inf_nan_num<5){
                    std::cout<<"NanInf"<<cv::Point2d(v,u)<<disp<<std::endl;
                }
            }
        }
    }else  if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float disp = disp_img.at<cv::Vec3f>(v,u)[1];
                if(isnan(disp)){
                    vec_nan.push_back(cv::Point(u,v));
                    inf_nan_num++;
                }else if(isinf(disp)){
                    vec_inf.push_back(cv::Point(u,v));
                    inf_nan_num++;
                }else if(disp == 0){
                    vec_zero.push_back(cv::Point(u,v));
                    continue;
                }else{
                    continue;
                }
                if(inf_nan_num<5){
                    std::cout<<"NanInf"<<cv::Point2d(v,u)<<disp<<std::endl;
                }
            }
        }
    }
    std::cout<<"The num of inf nan zero "<<cv::Point2d(disp_img.rows*disp_img.cols,inf_nan_num)<<cv::Point3d(vec_nan.size(),vec_inf.size(),vec_zero.size())<<std::endl;

    cv::Mat disp_check_img = cv::Mat::zeros(disp_img.size(),CV_8UC3);
    for (int v = 0; v < disp_img.rows; v++){
        for (int u = 0; u < disp_img.cols; u++) {
            int disp = disp_vis_img.at<uchar>(v,u);
            disp_check_img.at<cv::Vec3b>(v,u)[0] = disp;
            disp_check_img.at<cv::Vec3b>(v,u)[1] = disp;
            disp_check_img.at<cv::Vec3b>(v,u)[2] = disp;
        }
    }

    for(int i =0;i<vec_inf.size();i++){
        cv::Scalar color(0, 0, 255);
        cv::circle(disp_check_img, vec_inf[i], 1.0, color, 2);
    }

    for(int i =0;i<vec_nan.size();i++){
        cv::Scalar color(0, 255, 0);
        cv::circle(disp_check_img, vec_nan[i], 1.0, color, 2);
    }

    for(int i =0;i<vec_zero.size();i++){
        cv::Scalar color(255, 0, 0);
        cv::circle(disp_check_img, vec_zero[i], 1.0, color, 2);
    }

    cv::imshow("disp_check_img",disp_check_img);
    cv::waitKey();
}

cv::Mat polarimetricSynthesizer::validDisparity(const cv::Mat &disp_img){
    cv::Mat valid_img = cv::Mat::zeros(disp_img.size(),CV_8UC1);
    float max_disp = disp_img.cols/2.0;
    float min_disp = 0.01;
    if(disp_img.type() == CV_32FC3){
        for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float disp = disp_img.at<cv::Vec3f>(v,u)[1];
                if(disp>min_disp&&disp<max_disp){
                    valid_img.at<uchar>(v,u) = 255;
                }else{
                    valid_img.at<uchar>(v,u) = 0;
                }
            }
        }
    }else if(disp_img.type() == CV_32FC1){
         for (int v = 0; v < disp_img.rows; v++){
            for (int u = 0; u < disp_img.cols; u++) {
                float disp = disp_img.at<float>(v,u);
                if(disp>min_disp&&disp<max_disp){
                    valid_img.at<uchar>(v,u) = 255;
                }else{
                    valid_img.at<uchar>(v,u) = 0;
                }
            }
        }
    }
    return valid_img;
}

void polarimetricSynthesizer::showStereoMatch(const cv::Mat &disp_img,const cv::Mat &left_img,const cv::Mat &right_img){
    vector<cv::DMatch> matches;
    vector<cv::KeyPoint> keypoints1;
    vector<cv::KeyPoint> keypoints2;
    vector<cv::Point> pts;

    int match_size = 10;
    int rows = disp_img.rows;
    int cols = disp_img.cols;

    for(int i = 0;i<match_size;i++){
        int x = round(((double)rand()) / ((double)RAND_MAX) * (cols-1));
        int y = round(((double)rand()) / ((double)RAND_MAX) * (rows-1));
        pts.push_back(cv::Point(x,y));
    }

    for(int i = 0;i<pts.size();i++ ){
        int x = pts[i].x;
        int y = pts[i].y;
        float d = disp_img.at<ushort>(y,x);
        if(d==0){
            i--;
            continue;
        }
        d /= 100.0;
        int xr = round(x - d);
        cv::DMatch m;
        m.queryIdx = i;
        m.trainIdx =  i;
        matches.push_back(m);

        cv::KeyPoint kp1;
        cv::KeyPoint kp2;
        kp1.pt.x = x;
        kp1.pt.y = y;
        kp2.pt.x = xr;
        kp2.pt.y = y;
        keypoints1.push_back(kp1);
        keypoints2.push_back(kp2);
    }

    cv::Mat img_matches;
    cv::drawMatches(left_img, keypoints1, right_img, keypoints2, matches, img_matches);
    cv::imshow("img_matches",img_matches);
    cv::waitKey();
}

void polarimetricSynthesizer::showStereoOffset(const cv::Mat &disp_img,const cv::Mat &left_img,const cv::Mat &right_img,bool flag, std::string fix){
    cv::Mat img_offset = cv::Mat::zeros(left_img.rows,left_img.cols,CV_8UC3);
    if(disp_img.type() == CV_16UC1){
        bool isLeft = flag;
        for (int v = 0; v < left_img.rows; v++){
            for (int u = 0; u < left_img.cols ; u++) {
                if(isLeft){
                    float d = disp_img.at<ushort>(v,u);
                    if(d==0){
                        continue;
                    }
                    d /= 100.0;
                    int u2  = round(u - d);
                    if(u2<0){
                        continue;
                    }
                    // img_offset.at<cv::Vec3b>(v,u2) = left_img.at<cv::Vec3b>(v,u);
                    img_offset.at<cv::Vec3b>(v,u2)[0] = left_img.at<cv::Vec3b>(v,u)[0];
                    img_offset.at<cv::Vec3b>(v,u2)[1] = right_img.at<cv::Vec3b>(v,u2)[1];
                }else{
                    float d2 = disp_img.at<ushort>(v,u);
                    if(d2==0){
                        continue;
                    }
                    d2 /= 100.0;
                    int u1  = u + d2;
                    if(u1>=left_img.cols){
                        continue;
                    }
                    img_offset.at<cv::Vec3b>(v,u1)[2] = right_img.at<cv::Vec3b>(v,u)[2];
                    img_offset.at<cv::Vec3b>(v,u1)[0]  = left_img.at<cv::Vec3b>(v,u1)[0];
                }
            }
        }
    }else if(disp_img.type() == CV_32FC1){
        double max_tmp = -10;
        double min_tmp = 10;
        bool isDolp = flag;
        for (int v = 0; v < left_img.rows; v++){
            for (int u = 0; u < left_img.cols ; u++) {
                float dd = disp_img.at<float>(v,u);
                if (dd < 0.01){
                    continue; // 为0表示没有测量到
                }
                int u2  = round(u - dd);
                if(u2<0){
                    continue;
                }
                if(left_img.type() == CV_32FC1){
                    if(isDolp){
                        float dolp_left_val = left_img.at<float>(v,u);
                        float dolp_right_val = right_img.at<float>(v,u2);

                        int left_gray =  round( dolp_left_val * 255);
                        int right_gray =  round( dolp_right_val * 255);

                        img_offset.at<cv::Vec3b>(v,u2)[0] = left_gray;
                        img_offset.at<cv::Vec3b>(v,u2)[1] = right_gray;

                        max_tmp = max_tmp>dolp_left_val?max_tmp:dolp_left_val;
                        min_tmp = min_tmp<dolp_left_val?min_tmp:dolp_left_val;
                    }else{
                        float aolp_left_val = left_img.at<float>(v,u);
                        float aolp_right_val = right_img.at<float>(v,u2);
                        int left_gray =  round( aolp_left_val / M_PI * 253.0);
                        int right_gray =  round( aolp_right_val / M_PI * 253.0);

                        img_offset.at<cv::Vec3b>(v,u2)[0] = left_gray;
                        img_offset.at<cv::Vec3b>(v,u2)[1] = right_gray;

                        max_tmp = max_tmp>aolp_left_val?max_tmp:aolp_left_val;
                        min_tmp = min_tmp<aolp_left_val?min_tmp:aolp_left_val;
                    }
                }else if(left_img.type() == CV_8UC3){
                    img_offset.at<cv::Vec3b>(v,u2)[0] = left_img.at<cv::Vec3b>(v,u)[0];
                    img_offset.at<cv::Vec3b>(v,u2)[1] = right_img.at<cv::Vec3b>(v,u2)[1];
                }
            }
        }
        std::cout<<fix<<" max_tmp "<<max_tmp<<std::endl;
        std::cout<<fix<<" min_tmp "<<min_tmp<<std::endl;
    }

    cv::imshow("img_offset"+fix,img_offset);
    cv::waitKey();
    std::cout<<"showStereoOffset"<<std::endl;
}

void polarimetricSynthesizer::setNormal(const cv::Mat &norm_gt_img,cv::Mat &normal_img){
    std::cout<<"set normal"<<std::endl;
    // normal_img = norm_gt_img.clone();
    for (int v = 0; v < norm_gt_img.rows; v++){
        for (int u = 0; u < norm_gt_img.cols; u++) {
            float nx = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[0] - 1;
            float ny = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[1] - 1;
            float nz = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[2] - 1;

            // normal_img.at<cv::Vec3f>(v,u)[0] = -nz;
            // normal_img.at<cv::Vec3f>(v,u)[1] = -ny;
            // normal_img.at<cv::Vec3f>(v,u)[2] = -nx;

            normal_img.at<cv::Vec3f>(v,u)[0] = nz;
            normal_img.at<cv::Vec3f>(v,u)[1] = ny;
            normal_img.at<cv::Vec3f>(v,u)[2] = nx;

            // normal_img.at<cv::Vec3f>(v,u)[0] = nx;
            // normal_img.at<cv::Vec3f>(v,u)[1] = ny;
            // normal_img.at<cv::Vec3f>(v,u)[2] = nz;
        }
    }
}

void polarimetricSynthesizer::showNormal(const cv::Mat &normal_img,std::string suffix){

    cv::Mat normal_vis_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_8UC3);

    for (int v = 0; v < normal_img.rows; v++){
        for (int u = 0; u < normal_img.cols; u++) {
            float nx = normal_img.at<cv::Vec3f>(v,u)[0];
            float ny = normal_img.at<cv::Vec3f>(v,u)[1];
            float nz = normal_img.at<cv::Vec3f>(v,u)[2];

            int nr = round(255 * (nx + 1.0)/ 2.0);
            int ng = round(255 * (ny + 1.0)/ 2.0);
            int nb = round(255 * (nz + 1.0)/ 2.0);

            normal_vis_img.at<cv::Vec3b>(v,u)[0] = nr;
            normal_vis_img.at<cv::Vec3b>(v,u)[1] = ng;
            normal_vis_img.at<cv::Vec3b>(v,u)[2] = nb;
        }
    }
    std::cout<<"normal_img size : "<<normal_img.size()<<std::endl;
    cv::imshow("normal_vis_img"+suffix,normal_vis_img);
    cv::waitKey();
}

void polarimetricSynthesizer::showNormGT(const cv::Mat &norm_gt_img){
    std::cout<<"norm_gt_img size : "<<norm_gt_img.size()<<std::endl;
    std::cout<<"norm_gt_img channels : "<<norm_gt_img.channels()<<std::endl;
    std::cout<<"norm_gt_img type : "<<norm_gt_img.type()<<std::endl;
    std::cout<<"CV_16UC1 : "<<CV_16UC1<<std::endl;
    std::cout<<"CV_32FC1 : "<<CV_32FC1<<std::endl;
    std::cout<<"CV_8UC3 : "<<CV_8UC3<<std::endl;
    std::cout<<"CV_32FC3 : "<<CV_32FC3<<std::endl;

    cv::Mat norm_gt_vis_img = cv::Mat::zeros(norm_gt_img.rows,norm_gt_img.cols,CV_8UC3);
    if(0){
        for (int v = 0; v < norm_gt_img.rows; v++){
            for (int u = 0; u < norm_gt_img.cols; u++) {
                float nx = norm_gt_img.at<cv::Vec3f>(v,u)[0];
                float ny = norm_gt_img.at<cv::Vec3f>(v,u)[1];
                float nz = norm_gt_img.at<cv::Vec3f>(v,u)[2];

                int nr = round(255 * nx );
                int ng = round(255 * ny);
                int nb = round(255 * nz);

                norm_gt_vis_img.at<cv::Vec3b>(v,u)[0] = nr;
                norm_gt_vis_img.at<cv::Vec3b>(v,u)[1] = ng;
                norm_gt_vis_img.at<cv::Vec3b>(v,u)[2] = nb;
            }
        }
    }

    if(1){
        for (int v = 0; v < norm_gt_img.rows; v++){
            for (int u = 0; u < norm_gt_img.cols; u++) {
                float nx = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[0] - 1;
                float ny = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[1] - 1;
                float nz = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[2] - 1;

                float nx2 =  -nz;
                float ny2 = -ny;
                float nz2 = -nx;

                int nr = round(255 * (nx2 + 1.0)/ 2.0);
                int ng = round(255 * (ny2 + 1.0)/ 2.0);
                int nb = round(255 * (nz2 + 1.0)/ 2.0);

                norm_gt_vis_img.at<cv::Vec3b>(v,u)[0] = nr;
                norm_gt_vis_img.at<cv::Vec3b>(v,u)[1] = ng;
                norm_gt_vis_img.at<cv::Vec3b>(v,u)[2] = nb;
            }
        }
    }

    cv::imshow("norm_gt_vis_img",norm_gt_vis_img);
    cv::imshow("norm_gt_img",norm_gt_img);
    cv::waitKey();
}

void polarimetricSynthesizer::compareNormGT(const cv::Mat &norm_gt_img,const cv::Mat &normal_img,bool tranformed){
    vector<cv::Point> pts;
    pts.push_back(cv::Point(200,400));
    pts.push_back(cv::Point(300,500));
    pts.push_back(cv::Point(600,200));
    for(int i = 0;i<pts.size();i++){
        int u = pts[i].x;
        int v = pts[i].y;
        cv::Vec3f n_gt =  norm_gt_img.at<cv::Vec3f>(v,u);
        cv::Vec3f n_dp = normal_img.at<cv::Vec3f>(v,u);
        cv::Point3d n_gt0(n_gt[0],n_gt[1],n_gt[2]);
        cv::Point3d nn_gt(2*n_gt[0]-1,2*n_gt[1]-1,2*n_gt[2]-1);
        cv::Point3d nn_dp(n_dp[0],n_dp[1],n_dp[2]);
        if(!tranformed){
            std::cout <<n_gt0<<n_gt0.x*n_gt0.x+n_gt0.y*n_gt0.y+n_gt0.z*n_gt0.z
                                <<nn_gt<<nn_gt.x*nn_gt.x+nn_gt.y*nn_gt.y+nn_gt.z*nn_gt.z
                                <<nn_dp<<nn_dp.x*nn_dp.x+nn_dp.y*nn_dp.y+nn_dp.z*nn_dp.z<<std::endl;
        }else{
            std::cout <<n_gt0<<n_gt0.x*n_gt0.x+n_gt0.y*n_gt0.y+n_gt0.z*n_gt0.z
                                <<nn_dp<<nn_dp.x*nn_dp.x+nn_dp.y*nn_dp.y+nn_dp.z*nn_dp.z<<std::endl;
        }
    }

    // float min_depth;
    // float max_depth;

    // getDepthBound(depth_img,min_depth,max_depth);

    vector<cv::Point3d> qs1;
    vector<cv::Point3d> qs2;

    for (int v = 0; v < normal_img.rows; v++){
        for (int u = 0; u < normal_img.cols ; u++) {
            cv::Vec3f n_gt =  norm_gt_img.at<cv::Vec3f>(v,u);
            cv::Vec3f n_dp = normal_img.at<cv::Vec3f>(v,u);
            cv::Point3d n_gt0(n_gt[0],n_gt[1],n_gt[2]);
            cv::Point3d nn_dp(n_dp[0],n_dp[1],n_dp[2]);
            cv::Point3d nn_gt(2*n_gt[0]-1,2*n_gt[1]-1,2*n_gt[2]-1);
            if(!tranformed){
                qs1.push_back(nn_dp);
                qs2.push_back(nn_gt);
            }else{
                qs1.push_back(nn_dp);
                qs2.push_back(n_gt0);
            }
        }
    }

    Eigen::Vector3d qq(0, 0, 0);
    qq.normalize();
    std::cout<<qq.transpose()<<std::endl;

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < qs1.size(); i++) {
        Eigen::Vector3d q1(qs1[i].x, qs1[i].y, qs1[i].z);
        Eigen::Vector3d q2(qs2[i].x, qs2[i].y, qs2[i].z);
        q1.normalize();
        q2.normalize();
        // std::cout<<q1.transpose()<<" "<<q2.transpose()<<std::endl;
        W += q1 * q2.transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d RR = U * (V.transpose());
    if (RR.determinant() < 0) {
        RR = -RR;
    }
    std::cout << "RR="<<std::endl << RR << std::endl;
    std::cout<<cv::Point3d(1,0,0)<<std::endl<<RR*Eigen::Vector3d(1, 0, 0)<<std::endl;
    std::cout<<cv::Point3d(0,1,0)<<std::endl<<RR*Eigen::Vector3d(0, 1, 0)<<std::endl;
    std::cout<<cv::Point3d(0,0,1)<<std::endl<<RR*Eigen::Vector3d(0, 0, 1)<<std::endl;
}

void polarimetricSynthesizer::showDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string suffix){

    cv::Mat dop_vis_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_8UC1);
    cv::Mat aop_vis_img = cv::Mat::zeros(AoLP_img.rows,AoLP_img.cols,CV_8UC3);

    cv::Mat lp_vis_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_8UC3);

    for (int v = 1; v < DoLP_img.rows - 1; v++){
        for (int u = 1; u < DoLP_img.cols - 1; u++) {
            float rho = DoLP_img.at<float>(v,u);
            float phi = AoLP_img.at<float>(v,u);
            dop_vis_img.at<uchar>(v,u) = DoLP2Gray(rho);
            aop_vis_img.at<cv::Vec3b>(v,u) = AoLP2Color(phi);
            lp_vis_img.at<cv::Vec3b>(v,u) = DAoLP2Color(rho,phi);
        }
    }

    std::cout<<"DoLP_img size : "<<DoLP_img.size()<<std::endl;
    std::cout<<"AoLP_img size : "<<AoLP_img.size()<<std::endl;

    cv::imshow("dop_vis_img"+suffix,dop_vis_img);
    cv::imshow("aop_vis_img"+suffix,aop_vis_img);
    cv::imshow("lp_vis_img"+suffix,lp_vis_img);
    std::cout<<"CXY "<<camera_cx<<" "<<camera_cy<<std::endl;
    cv::imwrite("/home/tcr/pro_toolkit/depth2polarimetric/output/dop_vis_img"+suffix+".png",dop_vis_img);
    cv::imwrite("/home/tcr/pro_toolkit/depth2polarimetric/output/aop_vis_img"+suffix+".png",aop_vis_img);

    cv::waitKey();

    return;
}

void polarimetricSynthesizer::saveValid(const cv::Mat& valid_img,std::string syn_dir,std::string view_str, std::string fix){
    std::string prefix;
    if(reflection_type == DIFFUSE){
        prefix = "v";
    }else if(reflection_type == SPECULAR){
        prefix = "v";
    }else if(reflection_type == RAND){
        prefix = "v";
    }else{
        prefix = "v";
    }

    std::string suffix = ".png";

    std::string valid_png= syn_dir + "/" + prefix  + fix + view_str + suffix;
    std::cout<<"valid_png"<<valid_png<<std::endl;
    cv::imwrite(valid_png,valid_img);
}

void polarimetricSynthesizer::savePngVisDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str){
    std::string ref_str;
    if(reflection_type == DIFFUSE){
        ref_str = "dif";
    }else if(reflection_type == SPECULAR){
        ref_str = "spe";
    }else if(reflection_type == RAND){
        ref_str = "rand";
    }

    std::string dolp_png_dir = syn_dir + "/"+ref_str+"_dolp"+view_str+".png";
    std::string aolp_png_dir = syn_dir + "/"+ref_str+"_aolp"+view_str+".png";
    std::string lp_png_dir = syn_dir + "/"+ref_str+"_lp"+view_str+".png";

    std::cout<<dolp_png_dir<<std::endl;
    std::cout<<aolp_png_dir<<std::endl;
    std::cout<<lp_png_dir<<std::endl;

    // return;

    cv::Mat dop_vis_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_8UC1);
    cv::Mat aop_vis_img = cv::Mat::zeros(AoLP_img.rows,AoLP_img.cols,CV_8UC3);
    cv::Mat lp_vis_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_8UC3);

    for (int v = 1; v < DoLP_img.rows - 1; v++){
        for (int u = 1; u < DoLP_img.cols - 1; u++) {
            float rho = DoLP_img.at<float>(v,u);
            float phi = AoLP_img.at<float>(v,u);
            dop_vis_img.at<uchar>(v,u) = DoLP2Gray(rho);
            aop_vis_img.at<cv::Vec3b>(v,u) = AoLP2Color(phi);
            lp_vis_img.at<cv::Vec3b>(v,u) = DAoLP2Color(rho,phi);
        }
    }

    cv::imwrite(dolp_png_dir,dop_vis_img);
    cv::imwrite(aolp_png_dir,aop_vis_img);
    cv::imwrite(lp_png_dir,lp_vis_img);
    return;
}

void polarimetricSynthesizer::savePfmDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str,std::string fix){
    std::string prefix;
    std::string dolp_prefix;
    std::string aolp_prefix;
    if(reflection_type == DIFFUSE){
        dolp_prefix = "ddp";
        aolp_prefix = "dap";
        prefix = "pd";
    }else if(reflection_type == SPECULAR){
        dolp_prefix = "sdp";
        aolp_prefix = "sap";
        prefix = "ps";
    }else if(reflection_type == RAND){
        dolp_prefix = "rdp";
        aolp_prefix = "rap";
        prefix = "pr";
    }

    std::string suffix = ".pfm";

    std::string dolp_pfm= syn_dir + "/" + dolp_prefix + fix + view_str + suffix;
    std::string aolp_pfm = syn_dir + "/" + aolp_prefix + fix + view_str + suffix;

    std::cout<<"dolp_pfm:    "<<dolp_pfm<<std::endl;
    std::cout<<"aolp_pfm:    "<<aolp_pfm<<std::endl;
    // return;

    savePFM(DoLP_img,dolp_pfm);
    savePFM(AoLP_img,aolp_pfm);
    return;
}

void polarimetricSynthesizer::saveExrDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str){
    std::string prefix;
    std::string dolp_prefix;
    std::string aolp_prefix;
    float reflection_num;
    if(reflection_type == DIFFUSE){
        dolp_prefix = "ddp";
        aolp_prefix = "dap";
        prefix = "pd";
        reflection_num = 1;
    }else if(reflection_type == SPECULAR){
        dolp_prefix = "sdp";
        aolp_prefix = "sap";
        prefix = "ps";
        reflection_num = 2;
    }else if(reflection_type == RAND){
        dolp_prefix = "rdp";
        aolp_prefix = "rap";
        prefix = "pr";
        reflection_num = 2;
    }


    std::string suffix = ".exr";

    if(1){
        std::string exr_dir = syn_dir + "/" + prefix + view_str + suffix;
        cv::Mat pol_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_32FC3);
        for (int v = 0; v < pol_img.rows; v++){
            for (int u = 0; u < pol_img.cols ; u++) {
                pol_img.at<cv::Vec2f>(v,u)[0] = DoLP_img.at<float>(v,u);
                pol_img.at<cv::Vec2f>(v,u)[1] = AoLP_img.at<float>(v,u);
                pol_img.at<cv::Vec2f>(v,u)[2] = reflection_num;
            }
        }

        std::cout<<"exr_dir:    "<<exr_dir<<std::endl;
        cv::imwrite(exr_dir,pol_img);
    }else{
        std::string aolp_exr = syn_dir + "/" + aolp_prefix + view_str + suffix;
        std::string dolp_exr = syn_dir + "/" + dolp_prefix + view_str + suffix;
        std::cout<<"aolp_exr:    "<<aolp_exr<<std::endl;
        std::cout<<"dolp_exr:    "<<dolp_exr<<std::endl;

        cv::imwrite(aolp_exr,AoLP_img);
        cv::imwrite(dolp_exr,DoLP_img);
    }
}

void polarimetricSynthesizer::savePngDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string syn_dir,std::string view_str){
    std::string prefix;
    std::string dolp_prefix;
    std::string aolp_prefix;
    if(reflection_type == DIFFUSE){
        dolp_prefix = "ddp";
        aolp_prefix = "dap";
    }else if(reflection_type == SPECULAR){
        dolp_prefix = "sdp";
        aolp_prefix = "sap";
    }else if(reflection_type == RAND){
        dolp_prefix = "rdp";
        aolp_prefix = "rap";
    }
    std::string suffix = ".png";

    if(0){
        std::string png_dir = syn_dir + "/" + prefix + view_str + suffix;
        cv::Mat pol_img = cv::Mat::zeros(DoLP_img.rows,DoLP_img.cols,CV_32FC2);
        for (int v = 0; v < pol_img.rows; v++){
            for (int u = 0; u < pol_img.cols ; u++) {
                pol_img.at<cv::Vec2f>(v,u)[0] = DoLP_img.at<float>(v,u);
                pol_img.at<cv::Vec2f>(v,u)[1] = AoLP_img.at<float>(v,u);
            }
        }
        std::cout<<"png_dir:    "<<png_dir<<std::endl;
        cv::imwrite(png_dir,pol_img);
    }else{
        std::string aolp_png = syn_dir + "/" + aolp_prefix + view_str + suffix;
        std::string dolp_png = syn_dir + "/" + dolp_prefix + view_str + suffix;
        std::cout<<"aolp_png:    "<<aolp_png<<std::endl;
        std::cout<<"dolp_png:    "<<dolp_png<<std::endl;
        cv::imwrite(aolp_png,AoLP_img);
       cv::imwrite(dolp_png,DoLP_img);
    }
}
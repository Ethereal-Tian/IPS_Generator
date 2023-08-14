#include "rgbd2polari.h"
#include "utils.h"
#include <algorithm>
#include <assert.h>
#include <sys/types.h>
#include <string>
float eta = 1.5;
#define DIFFUSE 1
#define SPECUALR 0
using namespace std;
using namespace cv;
#include "polar_processor.h"

const vector<int> zenith_set{15, 30, 45, 60, 75, 90};

struct coords_ratio{
    vector<vector<int>> coords;
    float ratio;
};

void randomDiffuseSpecularCoords(
    coords_ratio& item,
    cv::Mat& type_img
){
    float ratio = item.ratio;
    vector<vector<int>> coords = item.coords;
    int total_num = coords.size();
    int specular_num = 0;
    if (ratio!=-1) specular_num = int(total_num / (1+ratio));

    random_shuffle(coords.begin(), coords.end());
    for(int i = 0; i < specular_num; i++){
        vector<int> coord = coords[i];
        type_img.at<uchar>(coord[0], coord[1]) = 0;
    }
    for(int i = specular_num; i < coords.size(); i++){
        vector<int> coord = coords[i];
        type_img.at<uchar>(coord[0], coord[1]) = 1;
    }
}

void normal2DAoLP(
    const std::string dataset, const cv::Mat& normal_img,
    cv::Mat& DDoLP_img, cv::Mat& DAoLP_img, cv::Mat& SDoLP_img, cv::Mat& SAoLP_img, cv::Mat& zenith_img
){
    polarimetricSynthesizer ser;
    ser.readParameters("", dataset);

    cv::Mat azimuth_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    zenith_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);

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

            if (nz < 0){
                nx = -nx;
                ny = -ny;
                nz = -nz;
            }
            float zenith = acos(nz);
            float azimuth = atan2(ny,nx);

            if(azimuth <= 0) azimuth += M_PI;
            else if(azimuth >= M_PI) azimuth -= M_PI;

            azimuth_img.at<float>(v,u) = azimuth;
            zenith_img.at<float>(v,u) = zenith;
        }
    }

    DDoLP_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    DAoLP_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    SDoLP_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);
    SAoLP_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_32FC1);

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
            if (specular_rho < 0 && specular_rho > -1.0*1e-7) specular_rho = 0;
            else if(specular_rho < -1.0*1e-7) cout << "specular " << specular_rho << ' ' << zenith << ' ' << eta << endl;
            float diffuse_rho =  (pow((eta-1/eta),2)*sin(zenith)*sin(zenith))/
                                                            (2+2*eta*eta-pow((eta+1/eta),2)*sin(zenith)*sin(zenith) + 4*cos(zenith)*sqrt(eta*eta-sin(zenith)*sin(zenith)));
            if (diffuse_rho < 0 && diffuse_rho > -1.0*1e-7) diffuse_rho = 0;
            else if(diffuse_rho < -1.0*1e-7) cout << "diffuse " << diffuse_rho << ' ' << zenith << ' ' << eta << endl;
            float diffuse_phi;
            float specular_phi;

            if(azimuth >0 ){
                diffuse_phi = azimuth;
            }else{
                diffuse_phi = azimuth + M_PI;
            }

            if(azimuth >= M_PI_2){
                specular_phi = azimuth - M_PI / 2.0;
            }else if(azimuth >= -M_PI / 2.0){
                specular_phi = azimuth + M_PI / 2.0;
            }else{
                specular_phi = azimuth + M_PI * 3.0 / 2.0;
            }

            DDoLP_img.at<float>(v,u) = diffuse_rho;
            DAoLP_img.at<float>(v,u) = diffuse_phi;
            SDoLP_img.at<float>(v,u) = specular_rho;
            SAoLP_img.at<float>(v,u) = specular_phi;
        }
    }
}


void generateReflectionByLabeledType(
    const std::string dataset,
    const cv::Mat& normal_img,
    const cv::Mat& label_img,
    std::map<int, int> label_type,
    cv::Mat& DoLP_img, cv::Mat& AoLP_img, cv::Mat& reflect_img
){
    // normal to DDoLP, DAoLP, SDoLP, SAoLP
    cv::Mat DDoLP_img, DAoLP_img, SDoLP_img, SAoLP_img, zenith_img; // float
    normal2DAoLP(dataset, normal_img, DDoLP_img, DAoLP_img, SDoLP_img, SAoLP_img, zenith_img);

    cv::Mat type_img = cv::Mat::zeros(label_img.rows, label_img.cols, CV_8UC1); // uchar

    for(int i = 0; i < label_img.rows; i++){
        for(int j = 0; j < label_img.cols; j++){
            type_img.at<uchar>(i, j) = uchar(label_type[label_img.at<int>(i, j)]);
        }
    }
    DoLP_img = cv::Mat::zeros(DDoLP_img.rows,DDoLP_img.cols,CV_32FC1);
    AoLP_img = cv::Mat::zeros(DAoLP_img.rows,DAoLP_img.cols,CV_32FC1);
    reflect_img = cv::Mat::zeros(DDoLP_img.rows,DDoLP_img.cols,CV_8UC1);
    std::srand(time(NULL));

    for (int v = 0; v < DDoLP_img.rows; v++){
        for (int u = 0; u < DDoLP_img.cols; u++) {
            int type = type_img.at<uchar>(v, u);
            double rand_num = ((double)rand()) / ((double)RAND_MAX);
            if(type == 1){ // diffuse
                DoLP_img.at<float>(v,u) = DDoLP_img.at<float>(v,u);
                AoLP_img.at<float>(v,u) = DAoLP_img.at<float>(v,u);
                reflect_img.at<uchar>(v,u) = 255;

            }else if (type == 0){    //specular
                DoLP_img.at<float>(v,u) = SDoLP_img.at<float>(v,u);
                AoLP_img.at<float>(v,u) = SAoLP_img.at<float>(v,u);
                reflect_img.at<uchar>(v,u) = 0;
            }
        }
    }
    // visualizeNormal(normal_img, true, "");
    // showDAoLP(DDoLP_img, DAoLP_img, "diffuse");
    // showDAoLP(SDoLP_img, SAoLP_img, "specular");
    // showDAoLP(DoLP_img, AoLP_img, "origin");
    // cv::imshow("reflect type origin", reflect_img);
    cv::imwrite("../output/reflect_vis_img_origin.png",reflect_img);
}

void generate_single_syn_daolp_by_labeled_type(bool left = true){
    std::string root;
    std::string norm_root;
    std::string rgb_path;
    std::string disp_path;
    std::string norm_path;
    std::string label_path;
    std::string subset = "Office/ModernModularOffice";
    // std::string subset = "Home/ArchvizKitchen";
    std::string index = "10";
    if (left){
        root = "/mnt/nas_8/datasets/tiancr/IRS/"+subset;
        rgb_path =  "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"/l_"+index+".png";
        disp_path = "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"/d_"+index+".exr";
        norm_path = "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"/n_"+index+".exr";
        label_path = "/mnt/nas_8/group/weihong/PolarMVS/SynLabel/"+subset+"/semseg_L_"+index+".txt";
    }
    else{
        root = "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"";
        rgb_path = "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"/r_5.png";
        disp_path = "/mnt/nas_8/datasets/tiancr/IRS/"+subset+"/d_5.exr";
        norm_path = "/mnt/nas_8/datasets/tiancr/b/Render4/IRS/"+subset+"/n_5_R.exr";
        label_path = "/mnt/nas_8/group/weihong/PolarMVS/SynLabel/"+subset+"/semseg_R_5.txt";
    }

    std::string config_path = "/no_need";
    std::string dataset_type = "IRS";
    std::string label_type_path = "../label_type.txt";
    std::string output_root = "/mnt/nas_8/group/weihong/PolarMVS/SynRender_0723";
    bool pfm = false;

    cv::Mat rgb_img = cv::imread(rgb_path);
    cv::Mat disp_img;
    if(!pfm) disp_img = cv::imread(disp_path, -1);
    else loadPFM(disp_path);
    cv::Mat label_img = readLabelMat(label_path);

    cv::Mat normal_img;
    if (left){
        cv::Mat normal_gt_img = cv::imread(norm_path, -1);
        normal_img = normal_gt_img.clone();
        setNormal(normal_gt_img, normal_img);
    }else {
        normal_img = cv::imread(norm_path, -1);
    }
    std::map<int, int> label_type = readType(label_type_path);

    cv::Mat DoLP_img, AoLP_img, reflect_img;
    generateReflectionByLabeledType(
        dataset_type, normal_img, label_img, label_type,
        DoLP_img, AoLP_img, reflect_img);

    // Clip & Add Noise
    vector<cv::Mat> polars_img;
    DolpAolpToPolars(DoLP_img, AoLP_img, rgb_img, polars_img);
    for (int i = 0; i < 4; i++){
        cv::Mat polar_img = polars_img[i];
        // cv::imshow("polar_origin"+to_string(i), polar_img);
        cv::imwrite("../output/polar"+to_string(i)+"_vis_origin.png",polar_img);

        cv::Mat noise = cv::Mat::zeros(polar_img.rows, polar_img.cols, polar_img.type());
        int mean = 0;
        int std = 5;
        randn(noise, (mean, mean, mean), (std, std, std));
        polars_img[i] = polar_img - noise;
        // cv::imshow("polar_noise"+to_string(i), polars_img[i]);
        cv::imwrite("../output/polar"+to_string(i)+"_vis_noise.png",polars_img[i]);
    }
    cv::Mat DoLP_img_clip, AoLP_img_clip;
    PolarsToDolpAolp(polars_img,DoLP_img_clip,AoLP_img_clip);
    // showDAoLP(DoLP_img_clip, AoLP_img_clip, "clip");

    if (left){
        // saveVisualizedOneChannelImage(DoLP_img_clip, output_root+"/DoLP.png", 0, 1, true);
        // saveVisualizedOneChannelImage(AoLP_img_clip, output_root+"/AoLP.png", 0, M_PI, true);
        // cv::imwrite(output_root+"/reflect.png", reflect_img);

        cv::Mat polar = cv::Mat::zeros(DoLP_img_clip.rows, DoLP_img_clip.cols, CV_16UC3);
        for(int u = 0;u <polar.cols;u++){
            for(int v = 0;v<polar.rows;v++){
                float dolp = DoLP_img_clip.at<float>(v,u);
                float aolp = AoLP_img_clip.at<float>(v,u);
                int dp_st = std::round(dolp*10000);
                int ap_st = std::round(aolp*10000);
                polar.at<cv::Vec3w>(v,u)[0] = dp_st;
                polar.at<cv::Vec3w>(v,u)[1] = ap_st;
                polar.at<cv::Vec3w>(v,u)[2] = 0;
            }
        }
        // show_ColorMap();
        cv::imwrite(output_root+"/lpL_clip.png", polar);
        cout << "done" << endl;
    }
    else{
        // saveVisualizedOneChannelImage(DoLP_img_clip, output_root+"/DoLP_R.png", 0, 1);
        // saveVisualizedOneChannelImage(AoLP_img_clip, output_root+"/AoLP_R.png", 0, M_PI);
        // cv::imwrite(output_root+"/reflect_R.png", reflect_img);

        cv::Mat polar = cv::Mat::zeros(DoLP_img_clip.rows, DoLP_img_clip.cols, CV_16UC3);
        for(int u = 0;u <polar.cols;u++){
            for(int v = 0;v<polar.rows;v++){
                float dolp = DoLP_img_clip.at<float>(v,u);
                float aolp = AoLP_img_clip.at<float>(v,u);
                int dp_st = std::round(dolp*10000);
                int ap_st = std::round(aolp*10000);
                polar.at<cv::Vec3w>(v,u)[0] = dp_st;
                polar.at<cv::Vec3w>(v,u)[1] = ap_st;
                polar.at<cv::Vec3w>(v,u)[2] = 0;
            }
        }
        // show_ColorMap();
        cv::imwrite(output_root+"/lpR_clip.png", polar);
        cout << "done" << endl;
    }

}

int main(int argc, char* argv[]){
    generate_single_syn_daolp_by_labeled_type(true);
}
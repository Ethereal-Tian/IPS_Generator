#include "rgbd2polari.h"
std::string dataset_dir;
std::string save_dir;


void synthesizePolarimetricDataByDisparity(std::string data_list_str){
    std::string data_dir = dataset_dir + "/"+data_list_str;
    std::string syn_dir = save_dir + "/"+data_list_str;
    std::cout<<data_dir<<std::endl;
    std::cout<<syn_dir<<std::endl;

    polarimetricSynthesizer ser;
    std::string view_str = "1";
    std::string rgb_dir =  data_dir+"/im"+view_str+".png";
    std::string disp_dir =  data_dir+"/disp"+view_str+".pfm";
    std::string config_dir =  data_dir + "/calib.txt";
    ser.readParameters(config_dir,"ETH3D");
    bool pfm = true;

    // std::string rgb_dir =  data_dir+"/view1.png";
    // std::string disp_dir =  data_dir+"/disp1.png";
    // std::string config_dir =  data_dir + "/x.yaml";
    // readParameters(config_dir);
    // bool pfm = false;

    cv::Mat rgb_img = cv::imread(rgb_dir);
    cv::Mat disp_img;

    if(!pfm){
        disp_img  = cv::imread(disp_dir,-1);
    }else{
        disp_img = loadPFM(disp_dir);
    }

    if(0){
        rgb_img.resize(640,480);
        disp_img.resize(640,480);        
    }

    cv::Mat normal_img = ser.disparity2Normal(disp_img);
	
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

    cv::imwrite(syn_dir+"/norm.png",normal_vis_img);
}

int main(int argc, char **argv)
{
    dataset_dir = "/home/tcr/DataSets/Stereo/Middlebury/scenes2014";
    save_dir = "/home/tcr/pro_toolkit/depth2polarimetric/output";

    ifstream data_list_file(dataset_dir+"/data_list.txt");
	std::string data_list_str;

    while(!data_list_file.eof()){
        getline(data_list_file, data_list_str);
        if(data_list_str == ""){
            break;
        }
        // std::cout<<"#"<<data_list_str<<std::endl;
//    synthesizePolarimetricDataByDepth(data_list_str);
        synthesizePolarimetricDataByDisparity(data_list_str);
    }

    return 0;
}


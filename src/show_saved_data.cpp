#include "rgbd2polari.h"

std::string dataset_dir;
std::string save_dir;

void showData(std::string data_list_str){
    polarimetricSynthesizer ser;

    std::string dataset_dir;
    std::string data_dir;
    std::string config_dir;
    bool pfm = true;
    dataset_dir = "/mnt/nas_54/datasets/tiancr";
    data_dir = dataset_dir + "/IRS/Store/Supermarket";
    std::string syn_dir = dataset_dir + "/Render/IRS/Store/Supermarket";

// ConvenienceStore_Day
// ConvenienceStore
// Supermarket
// Supermarket_Dark


    cv::Mat disp_img;
    if(!pfm){
        std::string disp_dir =  data_dir+"/d_"+data_list_str+".exr";
        std::cout<<disp_dir<<std::endl;
        disp_img  = cv::imread(disp_dir, -1);
    }else{
        std::string disp_dir =  data_dir+"/d_"+data_list_str+".pfm";
        std::cout<<disp_dir<<std::endl;
        disp_img = loadPFM(disp_dir);
    }

    std::string rgb_dir = data_dir+"/l_"+data_list_str+".png";
    std::string rgb2_dir = data_dir+"/r_"+data_list_str+".png";
    std::cout<<rgb_dir<<std::endl;
    std::cout<<rgb2_dir<<std::endl;
    cv::Mat rgb_img = cv::imread(rgb_dir);
    cv::Mat rgb2_img = cv::imread(rgb2_dir);

    ser.showStereoOffset(disp_img,rgb_img,rgb2_img, true, "RGB");

    if(0){
        // std::string aolp_name = syn_dir + "/dap_" + data_list_str + ".exr";
        // std::string dolp_name = syn_dir + "/ddp_" + data_list_str + ".exr";
        std::string aolp_name = syn_dir + "/dap_" + data_list_str + ".png";
        std::string dolp_name = syn_dir + "/ddp_" + data_list_str + ".png";

        cv::Mat aolp_read = cv::imread(aolp_name, -1);
        cv::Mat dolp_read = cv::imread(dolp_name, -1);
        std::cout<<"aolp_read "<<aolp_read.size() << aolp_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"dolp_read "<<dolp_read.size() << dolp_read.type()<<"-"<<CV_32FC1<<std::endl;

        std::cout<<"dolp  r"<<dolp_read.at<float>(320,480)<<std::endl;
        std::cout<<"aolp  r"<<aolp_read.at<float>(320,480)<<std::endl;
        std::cout<<"dolp  r"<<dolp_read.at<float>(480,320)<<std::endl;
        std::cout<<"aolp  r"<<aolp_read.at<float>(480,320)<<std::endl;
    }else if(0){
        std::string pol_name = syn_dir + "/pd_" + data_list_str + ".exr";
        // std::string pol_name = syn_dir + "/pd_" + data_list_str + ".png";
        cv::Mat pol_read = cv::imread(pol_name, -1);
        std::cout<<"pol_read "<<pol_read.size() << pol_read.type()<< "-"<<CV_32FC3<<std::endl;

        std::cout<<"dolp  r"<<pol_read.at<cv::Vec3f>(320,480)[0]<<std::endl;
        std::cout<<"aolp  r"<<pol_read.at<cv::Vec3f>(320,480)[1]<<std::endl;
        std::cout<<"dolp  r"<<pol_read.at<cv::Vec3f>(480,320)[0]<<std::endl;
        std::cout<<"aolp  r"<<pol_read.at<cv::Vec3f>(480,320)[1]<<std::endl;
    }else{
        std::string aolp1_name = syn_dir + "/dap_" + data_list_str + ".pfm";
        std::string dolp1_name = syn_dir + "/ddp_" + data_list_str + ".pfm";
        std::string aolp2_name = syn_dir + "/dapR_" + data_list_str + ".pfm";
        std::string dolp2_name = syn_dir + "/ddpR_" + data_list_str + ".pfm";
        std::string aolp3_name = syn_dir + "/sap_" + data_list_str + ".pfm";
        std::string dolp3_name = syn_dir + "/sdp_" + data_list_str + ".pfm";
        std::string aolp4_name = syn_dir + "/sapR_" + data_list_str + ".pfm";
        std::string dolp4_name = syn_dir + "/sdpR_" + data_list_str + ".pfm";

        cv::Mat aolp1_read = loadPFM(aolp1_name);
        cv::Mat dolp1_read = loadPFM(dolp1_name);
        cv::Mat aolp2_read = loadPFM(aolp2_name);
        cv::Mat dolp2_read = loadPFM(dolp2_name);
        cv::Mat aolp3_read = loadPFM(aolp3_name);
        cv::Mat dolp3_read = loadPFM(dolp3_name);
        cv::Mat aolp4_read = loadPFM(aolp4_name);
        cv::Mat dolp4_read = loadPFM(dolp4_name);

        std::cout<<"aolp1_read "<<aolp1_read.size() << aolp1_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"dolp1_read "<<dolp1_read.size() << dolp1_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"aolp2_read "<<aolp2_read.size() << aolp2_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"dolp2_read "<<dolp2_read.size() << dolp2_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"aolp3_read "<<aolp3_read.size() << aolp3_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"dolp3_read "<<dolp3_read.size() << dolp3_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"aolp4_read "<<aolp4_read.size() << aolp4_read.type()<< "-"<<CV_32FC1<<std::endl;
        std::cout<<"dolp4_read "<<dolp4_read.size() << dolp4_read.type()<< "-"<<CV_32FC1<<std::endl;

        std::cout<<"diffuse aolp left (320,480) "<<aolp1_read.at<float>(320,480)<<std::endl;
        std::cout<<"diffuse dolp left (320,480)"<<dolp1_read.at<float>(320,480)<<std::endl;
        std::cout<<"diffuse aolp right (320,480) "<<aolp2_read.at<float>(320,480)<<std::endl;
        std::cout<<"diffuse dolp right (320,480)"<<dolp2_read.at<float>(320,480)<<std::endl;
        std::cout<<"specular aolp left (320,480) "<<aolp3_read.at<float>(320,480)<<std::endl;
        std::cout<<"specular dolp left (320,480)"<<dolp3_read.at<float>(320,480)<<std::endl;
        std::cout<<"specular aolp right (320,480) "<<aolp4_read.at<float>(320,480)<<std::endl;
        std::cout<<"specular dolp right (320,480)"<<dolp4_read.at<float>(320,480)<<std::endl;

        std::cout<<"diffuse aolp left (480,320) "<<aolp1_read.at<float>(480,320)<<std::endl;
        std::cout<<"diffuse dolp left (480,320)"<<dolp1_read.at<float>(480,320)<<std::endl;
        std::cout<<"diffuse aolp right (480,320) "<<aolp2_read.at<float>(480,320)<<std::endl;
        std::cout<<"diffuse dolp right (480,320)"<<dolp2_read.at<float>(480,320)<<std::endl;
        std::cout<<"specular aolp left (480,320) "<<aolp3_read.at<float>(480,320)<<std::endl;
        std::cout<<"specular dolp left (480,320)"<<dolp3_read.at<float>(480,320)<<std::endl;
        std::cout<<"specular aolp right (480,320) "<<aolp4_read.at<float>(480,320)<<std::endl;
        std::cout<<"specular dolp right (480,320)"<<dolp4_read.at<float>(480,320)<<std::endl;

        ser.showDAoLP(dolp1_read,aolp1_read,"DL");
        ser.showDAoLP(dolp2_read,aolp2_read,"DR");
        ser.showDAoLP(dolp3_read,aolp3_read,"SL");
        ser.showDAoLP(dolp4_read,aolp4_read,"SR");

        ser.showStereoOffset(disp_img,dolp1_read,dolp2_read,true,"DD");
        ser.showStereoOffset(disp_img,aolp1_read,aolp2_read,false,"DA");
        ser.showStereoOffset(disp_img,dolp3_read,dolp4_read,true,"SD");
        ser.showStereoOffset(disp_img,aolp3_read,aolp4_read,false,"SA");
    }
}


int main(int argc, char **argv)
{
    // std::ifstream data_list_file("/home/tcr/pro_toolkit/depth2polarimetric/list/Test");
    
    std::string list_name_o_o = "/home/tcr/pro_toolkit/depth2polarimetric/list/IRS_Store_";
    list_name_o_o = list_name_o_o +"Supermarket";
    // std::string list_name_o_o = "/home/tcr/pro_toolkit/depth2polarimetric/list/Middlebury2014";

    std::cout<<list_name_o_o<<std::endl;
    std::ifstream data_list_file(list_name_o_o);

    // /home/tcr/pro_toolkit/depth2polarimetric/list/IRS_Store_ConvenienceStore
    // /home/tcr/pro_toolkit/depth2polarimetric/list/IRS_Store_ConvenienceStore_Day
    // /home/tcr/pro_toolkit/depth2polarimetric/list/IRS_Store_Supermarket
    // /home/tcr/pro_toolkit/depth2polarimetric/list/IRS_Store_Supermarket_Dark
	std::string data_list_str;

    while(!data_list_file.eof()){
        std::getline(data_list_file, data_list_str);
        if(data_list_str == ""){
            break;
        }
        std::cout<<"#"<<data_list_str<<std::endl;
//    synthesizePolarimetricDataByDepth(data_list_str);
        showData(data_list_str);
        // synthesizePolarimetricDataByDisparity2(data_list_str);
    }

    return 0;
}


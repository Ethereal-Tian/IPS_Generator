#include "rgbd2polari.h"
std::string dataset_dir;
std::string save_dir;


void maskDisparity(std::string data_name, std::string data_list_str){
    polarimetricSynthesizer ser;
    bool pfm;

    std::string data_dir = dataset_dir + data_name;
    std::string syn_dir = dataset_dir + "/Render" + data_name;

    std::string config_dir;
    std::string disp_dir;
    std::string norm_dir;

    disp_dir =  data_dir+"/d_"+data_list_str+".exr";
    norm_dir = data_dir+"/n_"+data_list_str+".exr";
    config_dir =  data_dir + "xxx";
    ser.readParameters(config_dir,"IRS");

    cv::Mat disp_img;
    if(!pfm){
        std::cout<<disp_dir<<std::endl;
        disp_img  = cv::imread(disp_dir, -1);
    }else{
        std::cout<<disp_dir<<std::endl;
        disp_img = loadPFM(disp_dir);
    }

    cv::Mat dispR_img = ser.DisparityL2R(disp_img);
    cv::Mat valid_img, validR_img;

    valid_img = ser.validDisparity(disp_img);
    //validR_img = ser.validDisparity(dispR_img);

    ser.saveValid(valid_img,syn_dir, "_"+ data_list_str, "");
    //ser.saveValid(validR_img,syn_dir, "_"+ data_list_str,"R");
}

int main0(int argc, char **argv)
{
    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    return 0;
    if(argc != 4){
        std::cout<<"Err "<<argc<<std::endl;
        return 0;
    }

    Reflection_TYPE ref_type;
    std::string ref_type_str = argv[1];
    if(ref_type_str == "diffuse"){
        ref_type = DIFFUSE;
    }else if(ref_type_str == "specular"){
        ref_type = SPECULAR;
    }else{
         std::cout<<"ref_type "<<ref_type<<std::endl;
        return 0;
    }

    // ifstream data_list_file("/home/tcr/pro_toolkit/depth2polarimetric/list/Test");

    std::string data_name = argv[2];
    std::string list_name = argv[3];


    // ConvenienceStore_Day
    // ConvenienceStore
    // Supermarket
    // Supermarket_Dark

    // std::string data_name = "/IRS/Store/Supermarket";
    // std::string list_name = "IRS_Store_Supermarket";

    // std::string data_name = "/Middlebury2014";
    // std::string list_name = "Middlebury2014";

    std::string list_name_o_o = "/home/tcr/pro_toolkit/depth2polarimetric/list/";
    list_name_o_o = list_name_o_o + list_name;

    std::cout<<list_name_o_o<<std::endl;
    ifstream data_list_file(list_name_o_o);

	std::string data_list_str;

    while(!data_list_file.eof()){
        getline(data_list_file, data_list_str);
        if(data_list_str == ""){
            break;
        }
        std::cout<<"#"<<data_list_str<<std::endl;
        maskDisparity(data_name,data_list_str);
        // break;
    }

    return 0;
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
        maskDisparity(data_name, view_id);
        float percent = 10000* i / (view_e-view_s+1)/100.0;
        std::cout<<dataset_name<<"-"<<subset_name<<"\t"<<percent<<"%"<<std::endl;
        // if(i>3){break;}
    }
    return 0;
}

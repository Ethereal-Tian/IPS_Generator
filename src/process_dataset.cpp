#include "rgbd2polari.h"
std::string dataset_dir;
std::string save_dir;

void synthesizePolarimetricDataByDepth(std::string data_list_str,Reflection_TYPE reflect_type){
    std::string data_dir = dataset_dir + "/"+data_list_str;
    std::string syn_dir = save_dir + "/"+data_list_str;
    std::cout<<data_dir<<std::endl;
    std::cout<<syn_dir<<std::endl;

    polarimetricSynthesizer ser;
    std::string view_str = "1";
    std::string rgb_dir =  data_dir + "/color/"+view_str+".png";
    std::string depth_dir =  data_dir + "/depth/"+view_str+".pgm";
    std::string config_dir =  data_dir + "/x.yaml";

    ser.readParameters(config_dir,"middlebury");
    cv::Mat rgb_img = cv::imread(rgb_dir);
    cv::Mat depth_img  = cv::imread(depth_dir,-1);

    cv::Mat normal_img = ser.depth2Normal(depth_img);

    vector<cv::Point3d> colors;
    vector<cv::Point3d> points;
    vector<cv::Point3d> norms;

    ser.depth2Pointcloud(rgb_img,depth_img,points,colors);
    ser.normal2Pointcloud(normal_img,norms);

    cv::Mat DoLP_img;
    cv::Mat AoLP_img;
    
    ser.normal2DAoLP(normal_img, DoLP_img, AoLP_img, reflect_type);
    
    ser.savePfmDAoLP(DoLP_img, AoLP_img, syn_dir, "_"+ view_str);
    ser.savePngDAoLP(DoLP_img, AoLP_img, syn_dir, "_"+ view_str);
}

void synthesizePolarimetricDataByDisparity2(std::string data_name, std::string data_list_str,Reflection_TYPE reflect_type){

    std::string data_dir = dataset_dir + data_name + data_list_str;
    std::string syn_dir = dataset_dir + "/Render" + data_name +data_list_str;

    polarimetricSynthesizer ser;
    std::string view_str = "1";
    std::string rgb_dir =  data_dir+"/im1.png";
    std::string disp_dir =  data_dir+"/disp0.pfm";
    std::string dispR_dir =  data_dir+"/disp1.pfm";
    std::string config_dir =  data_dir + "/calib.txt";
    ser.readParameters(config_dir,"middlebury");
    bool pfm = true;

    // std::string rgb_dir =  data_dir+"/view1.png";
    // std::string disp_dir =  data_dir+"/disp1.png";
    // std::string config_dir =  data_dir + "/x.yaml";
    // readParameters(config_dir);
    // bool pfm = false;

    cv::Mat rgb_img = cv::imread(rgb_dir);

    cv::Mat disp_img;
    cv::Mat dispR_img;

    if(!pfm){
        disp_img  = cv::imread(disp_dir,-1);
        dispR_img  = cv::imread(dispR_dir,-1);
    }else{
        disp_img = loadPFM(disp_dir);
        dispR_img = loadPFM(dispR_dir);
    }

    cv::Mat normal_img = ser.disparity2Normal(disp_img);
    cv::Mat normalR_img = ser.disparity2Normal(dispR_img);
    
    cv::Mat DoLP_img;
    cv::Mat AoLP_img;
    
    cv::Mat DoLPR_img;
    cv::Mat AoLPR_img;

    ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,reflect_type);
    ser.normal2DAoLP(normalR_img,DoLPR_img,AoLPR_img,reflect_type);

    cv::Mat valid_img = ser.validDisparity(disp_img);
    cv::Mat validR_img = ser.validDisparity(dispR_img);

    ser.savePfmDAoLP(DoLP_img, AoLP_img, syn_dir, "", "L");
    ser.savePfmDAoLP(DoLPR_img, AoLPR_img, syn_dir, "", "R");

    ser.saveValid(valid_img,syn_dir, "", "L");
    ser.saveValid(validR_img,syn_dir, "", "R");

    std::cout<<"syn_dir " << syn_dir << std::endl;
    std::cout<<"data_list_str " << data_list_str << std::endl;
}

void synthesizePolarimetricDataByDisparity(std::string data_name, std::string data_list_str,Reflection_TYPE reflect_type){
    polarimetricSynthesizer ser;
    bool pfm;
    pfm = false;
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

    cv::Mat norm_gt_img = cv::imread(norm_dir, -1);
    cv::Mat normal_img = cv::Mat::zeros(norm_gt_img.rows,norm_gt_img.cols,CV_32FC3);
    ser.setNormal(norm_gt_img,normal_img);
    // ser.showNormal(normal_img,"N");

    cv::Mat DoLP_img;
    cv::Mat AoLP_img;
    
    ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,reflect_type);
    // ser.showDAoLP(DoLP_img,AoLP_img);

    cv::Mat normalR_img = ser.NormalL2RbyDisparity(normal_img,disp_img);
    cv::Mat DoLPR_img;
    cv::Mat AoLPR_img;
    ser.normal2DAoLP(normalR_img,DoLPR_img,AoLPR_img,reflect_type);

    cv::Mat dispR_img = ser.DisparityL2R(disp_img);
    cv::Mat valid_img, validR_img;
    if(0){
        valid_img = ser.validDisparity(disp_img);
        validR_img = ser.validDisparity(dispR_img);
    }
    
    if(1){
        // ser.saveExrDAoLP(DoLP_img,AoLP_img,syn_dir, "_" + data_list_str);
        // ser.savePngDAoLP(DoLP_img,AoLP_img,syn_dir, "_" + data_list_str);
        ser.savePfmDAoLP(DoLP_img, AoLP_img, syn_dir, "_" + data_list_str, "L");
        ser.savePfmDAoLP(DoLPR_img, AoLPR_img, syn_dir, "_" + data_list_str,"R");
        if(0){
            ser.saveValid(valid_img,syn_dir, "_"+ data_list_str, "L");
            ser.saveValid(validR_img,syn_dir, "_"+ data_list_str,"R");
        }
    }else{
        if(0){
            // std::string aolp_name = syn_dir + "/dap_" + data_list_str + ".exr";
            // std::string dolp_name = syn_dir + "/ddp_" + data_list_str + ".exr";
            std::string aolp_name = syn_dir + "/dap_" + data_list_str + ".png";
            std::string dolp_name = syn_dir + "/ddp_" + data_list_str + ".png";

            cv::Mat aolp_read = cv::imread(aolp_name, -1);
            cv::Mat dolp_read = cv::imread(dolp_name, -1);
            std::cout<<"aolp_read "<<aolp_read.size() << aolp_read.type()<< "-"<<CV_32FC1<<std::endl;
            std::cout<<"dolp_read "<<dolp_read.size() << dolp_read.type()<<"-"<<CV_32FC1<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"dolp  r"<<dolp_read.at<float>(320,480)<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"aolp  r"<<aolp_read.at<float>(320,480)<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"dolp  r"<<dolp_read.at<float>(480,320)<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"aolp  r"<<aolp_read.at<float>(480,320)<<std::endl;
        }else if(0){
            std::string pol_name = syn_dir + "/pd_" + data_list_str + ".exr";
            // std::string pol_name = syn_dir + "/pd_" + data_list_str + ".png";
            cv::Mat pol_read = cv::imread(pol_name, -1);
            std::cout<<"pol_read "<<pol_read.size() << pol_read.type()<< "-"<<CV_32FC3<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"dolp  r"<<pol_read.at<cv::Vec3f>(320,480)[0]<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"aolp  r"<<pol_read.at<cv::Vec3f>(320,480)[1]<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"dolp  r"<<pol_read.at<cv::Vec3f>(480,320)[0]<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"aolp  r"<<pol_read.at<cv::Vec3f>(480,320)[1]<<std::endl;
        }else{
            std::string aolp_name = syn_dir + "/dap_" + data_list_str + ".pfm";
            std::string dolp_name = syn_dir + "/ddp_" + data_list_str + ".pfm";

            cv::Mat aolp_read = loadPFM(aolp_name);
            cv::Mat dolp_read = loadPFM(dolp_name);

            std::cout<<"aolp_read "<<aolp_read.size() << aolp_read.type()<< "-"<<CV_32FC1<<std::endl;
            std::cout<<"dolp_read "<<dolp_read.size() << dolp_read.type()<<"-"<<CV_32FC1<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"dolp  r"<<dolp_read.at<float>(320,480)<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(320,480)<<std::endl;
            std::cout<<"aolp  r"<<aolp_read.at<float>(320,480)<<std::endl;

            std::cout<<"dolp  c"<<DoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"dolp  r"<<dolp_read.at<float>(480,320)<<std::endl;

            std::cout<<"aolp  c"<<AoLP_img.at<float>(480,320)<<std::endl;
            std::cout<<"aolp  r"<<aolp_read.at<float>(480,320)<<std::endl;

        }
    }
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
//    synthesizePolarimetricDataByDepth(data_list_str,ref_type);
        synthesizePolarimetricDataByDisparity(data_name,data_list_str,ref_type);
        // synthesizePolarimetricDataByDisparity2(data_name, data_list_str,ref_type);
        // break;
    }

    return 0;
}


int main(int argc, char **argv)
{

    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    // return 0;
    if(argc != 6){
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

    std::string dataset_name = argv[2];
    std::string subset_name = argv[3];
    std::string data_name = "/IRS/" + dataset_name + "/" + subset_name;
    
    std::string view_s_str = argv[4];
    std::string view_e_str = argv[5];

    int view_s = std::stoi(view_s_str);
    int view_e = std::stoi(view_e_str);
    for(int i=view_s;i<=view_e;i++){
        std::string view_id = std::to_string(i);
        synthesizePolarimetricDataByDisparity(data_name, view_id, ref_type);
        float percent = 10000* i / (view_e-view_s+1)/100.0;
        std::cout<<dataset_name<<"-"<<subset_name<<"\t"<<percent<<"%"<<std::endl;
        // if(i>3){break;}
    }

    return 0;
}


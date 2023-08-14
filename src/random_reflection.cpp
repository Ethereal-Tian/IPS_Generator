#include "rgbd2polari.h"

std::string dataset_dir;
std::string outset_dir;

void generate_rand_reflect(std::string data_name, std::string data_list_str){

    std::string data_dir = dataset_dir + data_name;
    std::string syn_dir = dataset_dir + "/Render" + data_name;
    std::string out_dir = outset_dir + "/Render" + data_name;

    std::string aolp1_name = syn_dir + "/dapL_" + data_list_str + ".pfm";
    std::string dolp1_name = syn_dir + "/ddpL_" + data_list_str + ".pfm";
    std::string aolp2_name = syn_dir + "/dapR_" + data_list_str + ".pfm";
    std::string dolp2_name = syn_dir + "/ddpR_" + data_list_str + ".pfm";
    std::string aolp3_name = syn_dir + "/sapL_" + data_list_str + ".pfm";
    std::string dolp3_name = syn_dir + "/sdpL_" + data_list_str + ".pfm";
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

    cv::Mat aolpL,dolpL,aolpR,dolpR,typeL,typeR;
    polarimetricSynthesizer ser;
    std::srand(time(0));
    ser.generateRandomReflection(dolp1_read, aolp1_read, dolp3_read, aolp3_read, dolpL, aolpL, typeL);
    ser.generateRandomReflection(dolp2_read, aolp2_read, dolp4_read, aolp4_read, dolpR, aolpR, typeR);


    std::string typeL_dir = out_dir + "/tL_" + data_list_str + ".png";
    std::string typeR_dir = out_dir + "/tR_" + data_list_str + ".png";

    cv::imwrite(typeL_dir,typeL);
    cv::imwrite(typeR_dir,typeR);

    std::cout<<typeL_dir<<std::endl;
    std::cout<<typeR_dir<<std::endl;

    ser.reflection_type = RAND;
    ser.savePfmDAoLP(dolpL, aolpL, out_dir, "_"+ data_list_str,"L");
    ser.savePfmDAoLP(dolpR, aolpR, out_dir, "_"+ data_list_str,"R");

}

int main0(int argc, char **argv)
{
    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    // std::ifstream data_list_file("/home/tcr/pro_toolkit/depth2polarimetric/list/Test");

    if(argc != 3){
        std::cout<<"Err "<<argc<<std::endl;
        return 0;
    }

    std::string data_name = argv[1];
    std::string list_name = argv[2];

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
    std::ifstream data_list_file(list_name_o_o);


	std::string data_list_str;

    while(!data_list_file.eof()){
        std::getline(data_list_file, data_list_str);
        if(data_list_str == ""){
            break;
        }
        std::cout<<"#"<<data_list_str<<std::endl;
        generate_rand_reflect(data_name, data_list_str);
    }

    return 0;
}

int main(int argc, char **argv)
{

    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    outset_dir = "/mnt/nas_54/datasets/tiancr";

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
        generate_rand_reflect(data_name, view_id);
        float percent = 10000* i / (view_e-view_s+1)/100.0;
        std::cout<<dataset_name<<"-"<<subset_name<<"\t"<<percent<<"%"<<std::endl;
        // if(i>3){break;}
    }
    return 0;
}

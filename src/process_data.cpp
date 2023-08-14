#include "rgbd2polari.h"
std::string dataset_dir;
std::string vis_dir;


void synthesizePolarimetricDataByDisparity(std::string data_name){
    std::string data_dir = dataset_dir + data_name;
    std::string syn_dir = dataset_dir + "/Render" + data_name;
    std::string out_dir = vis_dir + "/Visualize" + data_name;
    std::cout<<data_dir<<std::endl;
    std::cout<<syn_dir<<std::endl;

    polarimetricSynthesizer ser;

    std::string config_dir =  data_dir + "/calib.txt";
    std::string rgb0_dir =  data_dir+"/im0.png";
    std::string rgb1_dir =  data_dir+"/im1.png";
    std::string disp0_dir =  data_dir+"/disp0.pfm";
    std::string disp1_dir =  data_dir+"/disp1.pfm";
    ser.readParameters(config_dir,"middlebury");
    bool pfm = true;

    // std::string config_dir =  data_dir + "xx";
    // std::string rgb0_dir =  data_dir+"/view1.png";
    // std::string rgb1_dir =  data_dir+"/view1.png";
    // std::string disp0_dir =  data_dir+"/../../disp1.png";
    // std::string disp1_dir =  data_dir+"/../../disp5.png";

    // ser.readParameters(config_dir,"middlebury2006");
    // bool pfm = false;
    

    // std::string config_dir =  data_dir + "/x";
    // std::string rgb0_dir =  data_dir+"/images/images_rig_cam4_undistorted/1477833684658155598.png";
    // std::string rgb1_dir =  data_dir+"/images/images_rig_cam5_undistorted/1477833684658155598.png";
    // std::string disp0_dir =  data_dir+"/depths/images_rig_cam4/1477833684658155598.pfm";
    // std::string disp1_dir =  data_dir+"/depths/images_rig_cam5/1477833684658155598.pfm";
    // ser.readParameters(config_dir,"middlebury2006");
    // bool pfm = true;

    std::string rap0_name = syn_dir + "/rap0.pfm";
    std::string rdp0_name = syn_dir + "/rdp0.pfm";
    std::string rap1_name = syn_dir + "/rap1.pfm";
    std::string rdp1_name = syn_dir + "/rdp1.pfm";

    std::string sap0_name = syn_dir + "/sap0.pfm";
    std::string sdp0_name = syn_dir + "/sdp0.pfm";
    std::string sap1_name = syn_dir + "/sap1.pfm";
    std::string sdp1_name = syn_dir + "/sdp1.pfm";

    std::string dap0_name = syn_dir + "/dap0.pfm";
    std::string ddp0_name = syn_dir + "/ddp0.pfm";
    std::string dap1_name = syn_dir + "/dap1.pfm";
    std::string ddp1_name = syn_dir + "/ddp1.pfm";

    std::string type0_name = syn_dir + "/t0.png";
    std::string type1_name = syn_dir + "/t1.png";

    std::string valid0_name = syn_dir + "/v0.png";
    std::string valid1_name = syn_dir + "/v1.png";


    // readParameters(config_dir);
    // bool pfm = false;

    cv::Mat rgb0_img = cv::imread(rgb0_dir);
    cv::Mat rgb1_img = cv::imread(rgb1_dir);

    cv::Mat disp0_img;
    cv::Mat disp1_img;

    if(!pfm){
        disp0_img  = cv::imread(disp0_dir,-1);
        disp1_img  = cv::imread(disp1_dir,-1);
    }else{
        disp0_img = loadPFM(disp0_dir);
        disp1_img = loadPFM(disp1_dir);
    }

    if(0){
        cv::imshow("rgb0_img",rgb0_img);
        ser.showDisparity(disp1_img);
        cv::waitKey();
    }

    cv::Mat valid0_img = ser.validDisparity(disp0_img);
    cv::Mat valid1_img = ser.validDisparity(disp1_img);

    cv::Mat norm0_img = ser.disparity2Normal(disp0_img);
    cv::Mat norm1_img = ser.disparity2Normal(disp1_img);

    cv::Mat dap0_img;
    cv::Mat ddp0_img;
    cv::Mat dap1_img;
    cv::Mat ddp1_img;

    ser.normal2DAoLP(norm0_img,ddp0_img,dap0_img,DIFFUSE);
    ser.normal2DAoLP(norm1_img,ddp1_img,dap1_img,DIFFUSE);

    ser.savePngVisDAoLP(ddp0_img,dap0_img, out_dir, "0");
    ser.savePngVisDAoLP(ddp1_img,dap1_img, out_dir, "1");

    cv::Mat sap0_img;
    cv::Mat sdp0_img;
    cv::Mat sap1_img;
    cv::Mat sdp1_img;

    ser.normal2DAoLP(norm0_img,sdp0_img,sap0_img,SPECULAR);
    ser.normal2DAoLP(norm1_img,sdp1_img,sap1_img,SPECULAR);

    ser.savePngVisDAoLP(sdp0_img,sap0_img, out_dir, "0");
    ser.savePngVisDAoLP(sdp1_img,sap1_img, out_dir, "1");

    cv::Mat rap0_img;
    cv::Mat rdp0_img;
    cv::Mat rap1_img;
    cv::Mat rdp1_img;

    cv::Mat type0_img,type1_img;
    std::srand(time(0));
    ser.generateRandomReflection(ddp0_img, dap0_img, sdp0_img, sap0_img, rdp0_img, rap0_img, type0_img);
    ser.generateRandomReflection(ddp1_img, dap1_img, sdp1_img, sap1_img, rdp1_img, rap1_img, type1_img);

    savePFM(dap0_img,dap0_name);
    savePFM(ddp0_img,ddp0_name);
    savePFM(dap1_img,dap1_name);
    savePFM(ddp1_img,ddp1_name);
    savePFM(sap0_img,sap0_name);
    savePFM(sdp0_img,sdp0_name);
    savePFM(sap1_img,sap1_name);
    savePFM(sdp1_img,sdp1_name);
    savePFM(rap0_img,rap0_name);
    savePFM(rdp0_img,rdp0_name);
    savePFM(rap1_img,rap1_name);
    savePFM(rdp1_img,rdp1_name);

    cv::imwrite(type0_name,type0_img);
    cv::imwrite(type1_name,type1_img);

    cv::imwrite(valid0_name,valid0_img);
    cv::imwrite(valid1_name,valid1_img);
}

// ./process_data /Middlebury/scenes2014/Adirondack-imperfect
int main(int argc, char **argv)
{
    dataset_dir = "/mnt/nas_8/datasets/tiancr";
    vis_dir = "/home/tcr/DataSets/Polar";

    if(argc != 2){
        std::cout<<"Err "<<argc<<std::endl;
        return 0;
    }
    std::string data_name = argv[1];
    synthesizePolarimetricDataByDisparity(data_name);
    return 0;
}


int main0(int argc, char **argv)
{
    // std::string type0_name =  "/mnt/nas_8/datasets/tiancr/Render/Middlebury/scenes2021/skates2/t0.png";
    // std::string type1_name = "/mnt/nas_8/datasets/tiancr/Render/Middlebury/scenes2021/chess1/t0.png";

    std::string type0_name =  "/mnt/nas_54/datasets/tiancr/Render/IRS/Home/ModernClassicInterior_NightFall_BL2/tL_3.png";
    std::string type1_name =  "/mnt/nas_54/datasets/tiancr/Render/IRS/Home/ModernClassicInterior_NightFall_BL2/tL_4.png";
    // std::string type1_name = "/mnt/nas_54/datasets/tiancr/Render/IRS/Office/OfficeMedley3/tL_3.png";

    cv::Mat type0_img =  cv::imread(type0_name,-1);
    cv::Mat type1_img = cv::imread(type1_name,-1) ;

    int a,b;

    a = type0_img.at<uchar>(100,200);
    b = type1_img.at<uchar>(100,200);
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;

    a = type0_img.at<uchar>(200,200);
    b = type1_img.at<uchar>(200,200);
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;

    a = type0_img.at<uchar>(200,100);
    b = type1_img.at<uchar>(200,100);
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;
    
    return 0;
}
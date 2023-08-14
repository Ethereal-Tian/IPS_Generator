#include "rgbd2polari.h"
bool showPointCloudFlag = false;

void synthesizePolarimetricDataByDepth(){
    polarimetricSynthesizer ser;
    bool pfm;
    std::string dataset_dir;
    std::string data_dir;
    std::string config_dir;
    std::string rgb_dir;
    std::string depth_dir;

    std::string rgb2_dir;
    std::string depth2_dir;


    if(0){
        dataset_dir = "/home/tcr/pro_toolkit/depth2polarimetric/data/joinMap";
        data_dir =  dataset_dir + "";
        rgb_dir =  data_dir + "/color/1.png";
        depth_dir =  data_dir + "/depth/1.pgm";
        config_dir =  data_dir + "/x.yaml";
        ser.readParameters(config_dir,"joinMap");
        pfm = false;
    }

    if(0){
        dataset_dir = "/mnt/nas_54/datasets/tiancr/BlendedMVS";
        data_dir = dataset_dir + "/dataset_low_res/5a3ca9cb270f0e3f14d0eddb";
        rgb_dir =  data_dir+"/blended_images/00000000.jpg";
        depth_dir =  data_dir+"/rendered_depth_maps/00000000.pfm";
        config_dir =  data_dir + "/cams/00000000_cam.txt";
        ser.readParameters(config_dir,"BlendedMVS");
        pfm = true;
    }

    if(0){
        dataset_dir = "/mnt/nas_54/datasets/tiancr/ETH3D";
        data_dir = dataset_dir + "/train/eth3d/electro";
        rgb_dir =  data_dir+"/images/images_rig_cam4_undistorted/1474975187520882738.png";
        depth_dir =  data_dir+"/depths/images_rig_cam4/1474975187520882738.pfm";
        config_dir =  data_dir + "/cams/00000001_cam.txt";
        ser.readParameters(config_dir,"ETH3D");
        pfm = true;
    }

    if(0){
        dataset_dir = "/mnt/nas_54/datasets/tiancr/DTU";
        data_dir = dataset_dir + "/train/mvs_training/dtu";
        rgb_dir =  data_dir+"/Rectified/scan2_train/rect_001_0_r5000.png";
        depth_dir =  data_dir+"/Depths/scan2_train/depth_map_0000.pfm";
        config_dir =  data_dir + "/Cameras/train/00000000_cam.txt";
        ser.readParameters(config_dir,"DTU");
        pfm = true;
    }
    if(1){
        // dataset_dir = "/home/tcr/DataSets/Polar";
        dataset_dir = "/home/tcr/pro_toolkit/depth2polarimetric/data/rgbp";
        data_dir = dataset_dir + "/images";
        rgb_dir =  data_dir+"/0_gt_rgb.png";
        depth_dir =  data_dir+"/0_gt_depth.png";
        // rgb_dir =  data_dir+"/gui_depth.png";
        // depth_dir =  data_dir+"/gui_depth.raw";

        config_dir =  data_dir + "/gui_depth.csv";
        ser.readParameters(config_dir,"Pericipio");
        pfm = false;
    }
    cv::Mat rgb_img = cv::imread(rgb_dir);
    // cv::Mat rgb2_img = cv::imread(rgb2_dir);
    cv::Mat depth_img;

    if(!pfm){
        // std::cout<<depth_dir<<std::endl;
        depth_img  = cv::imread(depth_dir, -1);
    }else{
        std::cout<<depth_dir<<std::endl;
        depth_img = loadPFM(depth_dir);
    }

    // cv::resize(rgb_img,rgb_img,depth_img.size());//DTU

    ser.showRGB(rgb_img);
    ser.showDepth(depth_img);

    cv::Mat normal_img = ser.depth2Normal(depth_img);
    ser.showNormal(normal_img);

    vector<cv::Point3d> colors;
    vector<cv::Point3d> points;
    vector<cv::Point3d> norms;

    if( showPointCloudFlag){
        ser.depth2Pointcloud(rgb_img,depth_img,points,colors);
        ser.normal2Pointcloud(normal_img,norms);
    }

    cv::Mat DoLP_img;
    cv::Mat AoLP_img;

    ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,DIFFUSE);
    // ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,SPECULAR);
    ser.showDAoLP(DoLP_img,AoLP_img);
}

void synthesizePolarimetricDataByDisparity(){
    std::cout<<"synthesizePolarimetricDataByDisparity"<<std::endl;
    polarimetricSynthesizer ser;
    bool pfm;
    std::string dataset_dir;
    std::string data_dir;
    std::string config_dir;
    std::string rgb_dir;
    std::string disp_dir;
    std::string norm_dir;

    std::string rgb2_dir;
    std::string disp2_dir;

    if(0){
        dataset_dir = "/home/tcr/DataSets/Stereo/Middlebury/scenes2006";
        data_dir = dataset_dir + "/FullSize/Aloe";
        rgb_dir =  data_dir+"/Illum2/Exp1/view1.png";
        disp_dir =  data_dir+"/disp1.png";
        config_dir =  data_dir + "";
        ser.readParameters(config_dir,"middlebury2006");
        pfm = false;
    }

    if(0){
        dataset_dir = "/home/tcr/pro_toolkit/depth2polarimetric/data/middlebury2014";
        data_dir = dataset_dir + "/Flowers-perfect";
        rgb_dir =  data_dir+"/im0.png";
        disp_dir =  data_dir+"/disp0.pfm";
        config_dir =  data_dir + "/calib.txt";
        ser.readParameters(config_dir,"middlebury");
        pfm = true;
    }

    if(0){
        dataset_dir = "/mnt/nas_54/datasets/tiancr/SceneFlow/FlyingThings3D";
        data_dir = dataset_dir + "";
        rgb_dir =  data_dir+"/frames_cleanpass/TRAIN/A/0000/left/0010.png";
        disp_dir =  data_dir+"/disparity/TRAIN/A/0000/left/0010.pfm";
        config_dir = "";
        ser.readParameters(config_dir,"FlyingThings3D");
        pfm = true;
    }

    if(0){
        rgb_dir = "/home/tcr/Datasets/KITTI/stereo2015/training/image_2/000000_10.png";
        disp_dir = "/home/tcr/Datasets/KITTI/stereo2015/training/disp_occ_0/000000_10.png";
        ser.readParameters(config_dir,"kitti2015");
        pfm = false;
    }

    if(0){
        // dataset_dir = "/home/tcr/DataSets/Stereo/InStereo2K";
        dataset_dir = "/mnt/nas_54/datasets/tiancr/InStereo2K";
        // data_dir = dataset_dir + "/part1/000000";
        // data_dir = dataset_dir + "/part1/000020";
        data_dir = dataset_dir + "/part2/000474";
        rgb_dir =  data_dir+"/left.png";
        rgb2_dir =  data_dir+"/right.png";
        disp_dir =  data_dir+"/left_disp.png";
        config_dir =  data_dir + "xxx";
        ser.readParameters(config_dir,"InStereo2K");
        pfm = false;
    }

    if(1){
        dataset_dir = "/mnt/nas_8/datasets/tiancr/IRS";
        data_dir = dataset_dir + "/Store/ConvenienceStore";
        //142
        rgb_dir =  data_dir+"/l_1.png";
        disp_dir =  data_dir+"/d_1.exr";
        norm_dir = data_dir+"/n_1.exr";
        config_dir =  data_dir + "xxx";
        ser.readParameters(config_dir,"IRS");
        pfm = false;
    }

    std::cout<<rgb_dir<<std::endl;
    cv::Mat rgb_img = cv::imread(rgb_dir);
    // cv::Mat rgb2_img = cv::imread(rgb2_dir);
    cv::Mat disp_img;

    if(!pfm){
        std::cout<<disp_dir<<std::endl;
        disp_img  = cv::imread(disp_dir, -1);
    }else{
        std::cout<<disp_dir<<std::endl;
        disp_img = loadPFM(disp_dir);
    }

    if(disp_img.type() == CV_8UC1){
        std::cout<<"disparity   "<<disp_img.at<uchar>(320,480)<<std::endl;
    }else if(disp_img.type() == CV_16UC1){
        std::cout<<"disparity   "<<disp_img.at<ushort>(320,480)<<std::endl;
    }else if(disp_img.type() == CV_32FC1){
        std::cout<<"disparity   "<<disp_img.at<float>(320,480)<<std::endl;
    }else if(disp_img.type() == CV_32FC3){
        cv::Vec3f ddd = disp_img.at<cv::Vec3f>(320,480);
        std::cout<<"disparity   "<<cv::Point3d(ddd[0],ddd[1],ddd[2])<<std::endl;
        ddd = disp_img.at<cv::Vec3f>(160,200);
        std::cout<<"disparity   "<<cv::Point3d(ddd[0],ddd[1],ddd[2])<<std::endl;
    }

    ser.showRGB(rgb_img);
    // ser.showDisparity(disp_img);
    ser.checkDisparity(disp_img);

    // ser.showStereoMatch(disp_img,rgb_img,rgb2_img);
    // ser.showStereoOffset(disp_img,rgb_img,rgb2_img);
    cv::Mat normal_img = ser.disparity2Normal(disp_img);
    ser.showNormal(normal_img);

    if(0){
        // The fx fy cx cy matters in the depth or disparity to the normal !!!
        ser.camera_fx = 480;
        ser.camera_fy = 480;
        ser.camera_cx = 480;
        ser.camera_cy = 270;
        ser.camere_bl = 1000;
        ser.camere_bf=  ser.camera_fx * ser.camere_bl / 100.0;

        cv::Mat normal_img = ser.disparity2Normal(disp_img);
        ser.showNormal(normal_img,"Change");
    }

    cv::Mat normal0_img;
    if(0){
        cv::Mat norm_gt_img = cv::imread(norm_dir, -1);
        ser.showNormGT(norm_gt_img);
        ser.compareNormGT(norm_gt_img,normal_img);
        normal0_img = normal_img.clone();
        ser.setNormal(norm_gt_img,normal_img);
        ser.showNormal(normal_img,"N");
        ser.compareNormGT(normal_img,normal0_img,true);
    }


    vector<cv::Point3d> colors;
    vector<cv::Point3d> points;
    vector<cv::Point3d> norms;
    ser.disparity2Pointcloud(rgb_img,disp_img,points,colors);
    ser.normal2Pointcloud(normal_img,norms);


    cv::Mat DoLP_img;
    cv::Mat AoLP_img;

    if(1){
        ser.normal2DAoLP(normal0_img,DoLP_img,AoLP_img,DIFFUSE);
    }

    ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,DIFFUSE);
    // ser.normal2DAoLP(normal_img,DoLP_img,AoLP_img,SPECULAR);
    ser.showDAoLP(DoLP_img,AoLP_img);

    if(1){
        std::cout<<"--------------------------------------"<<std::endl;
        std::cout<<"l_n[480][320]"<<normal0_img.at<cv::Vec3f>(480,320)[0]<<std::endl;
        std::cout<<"l_n[480][320]"<<normal0_img.at<cv::Vec3f>(480,320)[1]<<std::endl;
        std::cout<<"l_n[480][320]"<<normal0_img.at<cv::Vec3f>(480,320)[2]<<std::endl;
        std::cout<<"l_n[320][480]"<<normal0_img.at<cv::Vec3f>(320,480)[0]<<std::endl;
        std::cout<<"l_n[320][480]"<<normal0_img.at<cv::Vec3f>(320,480)[1]<<std::endl;
        std::cout<<"l_n[320][480]"<<normal0_img.at<cv::Vec3f>(320,480)[2]<<std::endl;
        std::cout<<"--------------------------------------"<<std::endl;


        std::cout<<"--------------------------------------"<<std::endl;
        std::cout<<"arr_aolp_L[480][320]"<<AoLP_img.at<float>(480,320)<<std::endl;
        std::cout<<"arr_aolp_L[320][480]"<<AoLP_img.at<float>(320,480)<<std::endl;
        std::cout<<"--------------------------------------"<<std::endl;
    }

    // Right camera Normal Dolp Aolp
    if(0){
        cv::Mat normalR_img = ser.NormalL2RbyDisparity(normal_img,disp_img);
        ser.showNormal(normalR_img,"R");

        cv::Mat DoLPR_img;
        cv::Mat AoLPR_img;

        ser.normal2DAoLP(normalR_img,DoLPR_img,AoLPR_img,DIFFUSE);
        // ser.normal2DAoLP(normalR_img,DoLPR_img,AoLPR_img,SPECULAR);
        ser.showDAoLP(DoLPR_img,AoLPR_img,"R");
    }

    // Valid
    if(0){
        cv::Mat dispR_img = ser.DisparityL2R(disp_img);
        cv::Mat valid_img = ser.validDisparity(disp_img);
        cv::Mat validR_img = ser.validDisparity(dispR_img);
        cv::imshow("valid_img.png",valid_img);
        cv::imshow("validR_img.png",validR_img);
        cv::waitKey();
    }

    std::cout<<"Finshed!"<<std::endl;
}

int main(int argc, char **argv)
{
    showPointCloudFlag = false;
    if(argc == 2){
        std::string str1 = argv[1];
        if(str1=="pc"){
           showPointCloudFlag = true;
        }
    }else{

    }

    if(1){
        Eigen::Vector3d X(1,0,0);
        Eigen::Vector3d Y(0,1,0);
        Eigen::Vector3d Z(0,0,1);
        Eigen::Vector3d ZZ = X.cross(Y);
        std::cout<< "Z  "<<Z.transpose()<<std::endl;
        std::cout<< "ZZ "<<ZZ.transpose()<<std::endl;

        std::cout<<"asin(1) "<<asin(1)<<std::endl;
        std::cout<<"asin(-1) "<<asin(-1)<<std::endl;

        std::cout<<"acos(1) "<<acos(1)<<std::endl;
        std::cout<<"acos(-1) "<<acos(-1)<<std::endl;

        std::cout<<"atan(1) "<<atan(1)<<std::endl;
        std::cout<<"atan(-1) "<<atan(-1)<<std::endl;

        std::cout<<"atan2(0.5,1) "<<atan2(0.5,1)<<std::endl;
        std::cout<<"atan2(-0.5,1) "<<atan2(-0.5,1)<<std::endl;
    }

    // synthesizePolarimetricDataByDepth();
    synthesizePolarimetricDataByDisparity();
    std::cout<<"End"<<std::endl;
    return 0;
}


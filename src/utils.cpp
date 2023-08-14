#include  "utils.h"

/* =======================================
 * ============= File IO =================
 * ======================================= */

std::vector<std::string> getFileNames(
    const std::string dir_path,
    std::string prefix,
    std::string suffix
){
    DIR *pDir;
    struct dirent* ptr;
    std::vector<string> filenames;
    std::string filename;
    std::string f_prefix;
    std::string f_suffix;    
    if(!(pDir = opendir(dir_path.c_str())))
    {
        cout << "Open Dir " << dir_path << " Failed" << endl;
        system("pause");
        return filenames;
    }
    while((ptr = readdir(pDir))!=0) {
        filename = ptr->d_name;            
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {            
            if (prefix != "")
            {
                f_prefix = filename.substr(0, prefix.size());                
                if(strcmp(prefix.c_str(), f_prefix.c_str()) != 0) continue;
            }
            if (suffix != "")
            {
                f_suffix = filename.substr(filename.size()-suffix.size(), suffix.size());                
                if(strcmp(suffix.c_str(), f_suffix.c_str()) != 0) continue;
            }                    
            filenames.push_back(filename);
            
        }
    }
    closedir(pDir);      
    sort(filenames.begin(), filenames.end());
    return filenames;
}

void checkAndCreateDir(const std::string DirPath){
    if (access(DirPath.c_str(), F_OK) == -1){
        std::string cmd = "mkdir -p " + DirPath;
        system(cmd.c_str());
    }
}

/* =======================================
 * ========= Math Calculation ============
 * ======================================= */

std::vector<float> solveQuartic(float a,float b,float c,float d,float e){
    std::vector<float> res;
    if(a!=1){
        b /= a;
        c /= a;
        d /= a;
        e /= a;
    }
    float q = c * c / 9.0 + e * 4.0 / 3.0 - b * d /3.0;
    float r = e*b*b / 2.0 - b*c*d / 6.0 + c * c * c / 27.0 - e*c * 4.0 / 3.0 + d*d / 2.0;

    float sqR = - c * 2.0/ 3.0 + b * b / 4.0  + 2.0 * std::sqrt(q) * std::cos(std::acos(r / std::sqrt(q * q * q)) / 3.0);

    float RRe = std::sqrt(sqR);
    float AA = b * b * 3.0 / 4.0 - 2.0 * c - sqR;
    float BB =  (-2.0 * d - b * b * b/ 4.0 + b * c) / RRe;

    float square_sin_zenith1 = -b / 4.0 - RRe / 2.0 + std::sqrt( AA - BB ) / 2.0;
    float square_sin_zenith2 = -b / 4.0 + RRe / 2.0 - std::sqrt( AA + BB ) / 2.0;

    res.push_back(square_sin_zenith1);
    res.push_back(square_sin_zenith2);
    return res;
}

float solveQuadratic(float a,float b,float c){
    if(a!=1){
        b /= a;
        c /= a;
    }
    float square_sin_zenith = -b / 2.0 + std::sqrt(b * b / 4.0 -  c);
    return  square_sin_zenith;
}

float diffuseZenithToRho(float zenith,float eta){
    float rho =  (pow((eta-1/eta),2)*sin(zenith)*sin(zenith))/ \
                    (2+2*eta*eta-pow((eta+1/eta),2)*sin(zenith)*sin(zenith) + 4*cos(zenith)*sqrt(eta*eta-sin(zenith)*sin(zenith)));
    return rho;
}

float specularZenithToRho(float zenith,float eta){
    float rho =  2*sin(zenith)*sin(zenith)*cos(zenith) * sqrt(eta*eta-sin(zenith)*sin(zenith))/ \
                (eta*eta - sin(zenith)*sin(zenith) - eta*eta * sin(zenith)*sin(zenith) + 2*sin(zenith)*sin(zenith)*sin(zenith)*sin(zenith));
    return rho;
}

float diffuseRhoToZenith(float rho,float eta){
    if (rho >=  (eta*eta-1)/(eta*eta+1) -1e-8){
        return M_PI / 2.0;
    }
    if(rho == 0){
        return 0;
    }

    float quad_a = pow(( pow((eta - 1/eta),2) + rho*pow((eta + 1/eta),2)),2) - 16*rho*rho;
    float quad_b = -4*(eta*eta + 1)*(rho*rho + rho)*pow((eta - 1/eta),2);
    float quad_c = 4*rho*rho*pow((eta*eta - 1),2);
    float root = solveQuadratic(quad_a,quad_b,quad_c);
    float zenith = asin( sqrt(root)) ;
    return zenith;
}

std::vector<float> specularRhoToZenith(float rho,float eta){
    std::vector<float> zeniths;
    if(rho == 0){
        zeniths.push_back(0.0);
        zeniths.push_back(M_PI / 2.0);
        return zeniths;
    }
    if(rho == 1.0){
        double zenith0 = asin(sqrt(eta*eta/(eta*eta+1)));
        zeniths.push_back(zenith0);
        zeniths.push_back(zenith0);
        return zeniths;
    }
     
    float quar_a = 4*(rho*rho - 1);
    float quar_b = 4*(-rho*rho + 1)*(eta*eta + 1);
    float quar_c = rho*rho*pow((eta*eta + 1),2) + 4*(rho*rho-1)*eta*eta;
    float quar_d = (-2*rho*rho*eta*eta*(eta*eta + 1));
    float quar_e =  pow(eta,4)*rho*rho;
    std::vector<float> roots = solveQuartic(quar_a,quar_b,quar_c,quar_d,quar_e);
    float zenith1 = asin( sqrt(roots[0]));
    float zenith2 = asin( sqrt(roots[1])) ;

    if(std::isnan(zenith1) && rho < 0.001){
        zenith1 = 0.0;
    }
    
    if(std::isnan(zenith2) && rho < 0.001){
        zenith2 = M_PI / 2.0;
    }

    if(std::isnan(zenith1)){
        std::cout<<"nan1 rho "<<rho<<std::endl;
    }
    
    if(std::isnan(zenith2)){
        std::cout<<"nan2 rho "<<rho<<std::endl;
    }
    
    zeniths.push_back(zenith1);
    zeniths.push_back(zenith2);

    return zeniths;
}

float diffuseAzimuthToPhi(float azimuth){
    float phi;
    if(azimuth >M_PI ){
        phi = azimuth - M_PI;
    }else if(azimuth <0){
        phi = azimuth + M_PI;
    }else{
        phi = azimuth;
    }
    return phi;
}

float specularAzimuthToPhi(float azimuth){
    float phi;
    if(azimuth > M_PI * 3.0 / 2.0){
        phi = azimuth - M_PI * 3.0 / 2.0;
    }else if(azimuth > M_PI / 2.0){
        phi = azimuth - M_PI / 2.0;
    }else if(azimuth > -M_PI / 2.0){
        phi = azimuth + M_PI / 2.0;
    }else{
        phi = azimuth + M_PI * 3.0 / 2.0;
    }
    return phi;
}

std::vector<float> diffusePhiToAzimuth(float phi){
    float azimuth1 = phi;
    // float azimuth2 = phi + M_PI; // [0,2pi]
    float azimuth2 = phi - M_PI; // [-pi,pi]
    std::vector<float> azimuths;
    azimuths.push_back(azimuth1);
    azimuths.push_back(azimuth2);
    return azimuths;
}

std::vector<float> specularPhiToAzimuth(float phi){
    float azimuth1 = phi + M_PI / 2.0;
    float azimuth2 = phi - M_PI / 2.0;
    // if(azimuth2<0){// [0,2pi]
    //     azimuth2 += 2*M_PI;
    // }
    if(azimuth1>M_PI){// [-pi,pi]
        azimuth1 -= 2*M_PI;
    }
    std::vector<float> azimuths;
    azimuths.push_back(azimuth1);
    azimuths.push_back(azimuth2);
    return azimuths;
}
     
/* =======================================
 * ============= Image Show ==============
 * ======================================= */

cv::Vec3b Normal2Color(cv::Vec3f normValue){
    float nx = normValue[0];
    float ny = normValue[1];
    float nz = normValue[2];
    // int red = round(255 * (nx + 1.0)/ 2.0);
    // int green = round(255 * (ny + 1.0)/ 2.0);
    // int blue = round(255 * (nz + 1.0)/ 2.0);

    // X: -1 to +1 :  Red:     0 to 255
    // Y: -1 to +1 :  Green:   0 to 255
    // Z:  0 to -1 :  Blue:  128 to 255

    int red = 0;
    int green = 0;
    int blue = 0;
    if(nz > 0){
        red = round(255 * (-nx + 1.0)/ 2.0);
        green = round(255 * (-ny + 1.0)/ 2.0);
        blue = round(255 * (nz + 1.0)/ 2.0);
    }else{
        red = round(255 * (nx + 1.0)/ 2.0);
        green = round(255 * (ny + 1.0)/ 2.0);
        blue = round(255 * (-nz + 1.0)/ 2.0);
    }
    return cv::Vec3b(blue,green,red);
}

cv::Vec3b DAoLP2Color(float dolpValue,float aolpValue){
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

cv::Vec3b AoLP2Color(float aolpValue){
        // aolpValue = M_PI / 2;
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

char DoLP2Gray(float dolpValue){
    // int gray_tmp =  dolpValue;
    int gray_tmp =  round( dolpValue * 255);
    char res = gray_tmp;
    return res;
} 

void showNormal(const cv::Mat normal, std::string name){
    cv::Mat color = cv::Mat::zeros(normal.rows, normal.cols, CV_8UC3);
    for(int i = 0; i < normal.rows; i++){
        for(int j = 0; j < normal.cols; j++){
            color.at<cv::Vec3b>(i, j) = Normal2Color(normal.at<cv::Vec3f>(i, j));
        }
    }
    cv::imshow(name, color);
}

void visualizeNormal(const cv::Mat normal_img, const bool show = false, const string path = ""){
    cv::Mat normal_vis_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_8UC3);
    for (int v = 0; v < normal_img.rows; v++){
        for (int u = 0; u < normal_img.cols; u++) {
            float nx = normal_img.at<cv::Vec3f>(v,u)[0];
            float ny = normal_img.at<cv::Vec3f>(v,u)[1];
            float nz = normal_img.at<cv::Vec3f>(v,u)[2];
            
            if(nz > 0){
                nx = -nx;
                ny = -ny;
                nz = -nz;
            }
            // X: -1 to +1 :  Red:     0 to 255
            // Y: -1 to +1 :  Green:   0 to 255
            // Z:  0 to -1 :  Blue:  128 to 255
  
            int nr = round(255 * (nx + 1.0)/ 2.0);
            int ng = round(255 * (ny + 1.0)/ 2.0);
            int nb = round(255 * (-nz + 1.0)/ 2.0);

            normal_vis_img.at<cv::Vec3b>(v,u)[0] = nb;
            normal_vis_img.at<cv::Vec3b>(v,u)[1] = ng;
            normal_vis_img.at<cv::Vec3b>(v,u)[2] = nr;
        }
    }

    if (show){
        cv::imshow("normal", normal_vis_img);
        cv::waitKey();
        cv::destroyAllWindows();
    }
    string img_name = path==""?"output":path;
    cv::imwrite("../output/"+img_name+".png",normal_vis_img);
}

void showDAoLP(const cv::Mat& DoLP_img,const cv::Mat& AoLP_img,std::string suffix){

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
    cv::imwrite("../output/dop_vis_img"+suffix+".png",dop_vis_img);
    cv::imwrite("../output/aop_vis_img"+suffix+".png",aop_vis_img);

    cv::waitKey();
    
    return;
}

void showColorMap(){    
    cv::Mat depth1_img = cv::Mat::zeros(200,200,CV_8UC1);
    cv::Mat depth2_img = cv::Mat::zeros(200,200,CV_8UC1);
    cv::Mat normal1_img = cv::Mat::zeros(200,200,CV_32FC3);
    cv::Mat normal2_img = cv::Mat::zeros(200,200,CV_32FC3);

    cv::Mat df_dolp1_img = cv::Mat::zeros(200,200,CV_32FC1);
    cv::Mat df_dolp2_img = cv::Mat::zeros(200,200,CV_32FC1);

    cv::Mat df_aolp1_img = cv::Mat::zeros(200,200,CV_32FC1);
    cv::Mat df_aolp2_img = cv::Mat::zeros(200,200,CV_32FC1);

    cv::Mat sp_dolp1_img = cv::Mat::zeros(200,200,CV_32FC1);
    cv::Mat sp_dolp2_img = cv::Mat::zeros(200,200,CV_32FC1);

    cv::Mat sp_aolp1_img = cv::Mat::zeros(200,200,CV_32FC1);
    cv::Mat sp_aolp2_img = cv::Mat::zeros(200,200,CV_32FC1);

    for (int v = 1; v < 200; v++){
        for (int u = 1; u < 200; u++) {
            double center_x = 100;
            double center_y = 100;
            double center_z = 100;
            double x = u;
            double y = v;
            double xx_yy = (x-center_x)*(x-center_x) + (y-center_y)* (y-center_y);
            if(sqrt(xx_yy)>100){
                continue;
            }
            double z1 = -sqrt(100*100 - xx_yy) + center_z;
            double z2 = sqrt(100*100 - xx_yy) + center_z;

            int w1 = 255 - z1/ 200.0* 255.0;// white = near && black = far 
            int w2 = 255 - z2/ 200.0* 255.0;// white = near && black = far 

            depth1_img.at<uchar>(v,u) = w1;
            depth2_img.at<uchar>(v,u) = w2;

            cv::Vec3f n1 = cv::Vec3f((x-center_x)/100.0,(y-center_y)/100.0,(z1-center_z)/100.0);
            cv::Vec3f n2 = cv::Vec3f(-(x-center_x)/100.0,-(y-center_y)/100.0,-(z2-center_z)/100.0);

            normal1_img.at<cv::Vec3f>(v,u) = n1;
            normal2_img.at<cv::Vec3f>(v,u) = n2;
            float eta = 1.5;

            float zenith1 = acos(-n1[2]);
            float azimuth1 = atan2(-n1[1],n1[0]);

            float diffuse_rho1 = diffuseZenithToRho(zenith1,eta);
            float specular_rho1 = specularZenithToRho(zenith1,eta);

            float diffuse_phi1 = diffuseAzimuthToPhi(azimuth1);
            float specular_phi1 = specularAzimuthToPhi(azimuth1);

            float zenith2 = acos(-n2[2]);
            float azimuth2 = atan2(-n2[1],n2[0]);

            float diffuse_rho2 = diffuseZenithToRho(zenith2,eta);
            float specular_rho2 = specularZenithToRho(zenith2,eta);

            float diffuse_phi2 = diffuseAzimuthToPhi(azimuth2);
            float specular_phi2 = specularAzimuthToPhi(azimuth2);

            if(u%100== 50 && v%100==50){
                std::cout<<cv::Point(u,v)<<cv::Point2d(zenith1,zenith2)<<cv::Point2d(azimuth1,azimuth2);
                std::cout<<cv::Point2d(diffuse_phi1,specular_phi1);
                std::cout<<cv::Point2d(diffuse_phi2,specular_phi2);
                std::cout<<std::endl;
            }

            df_dolp1_img.at<float>(v,u) = diffuse_rho1;
            df_aolp1_img.at<float>(v,u) = diffuse_phi1;

            sp_dolp1_img.at<float>(v,u) = specular_rho1;
            sp_aolp1_img.at<float>(v,u) = specular_phi1;

            df_dolp2_img.at<float>(v,u) = diffuse_rho2;
            df_aolp2_img.at<float>(v,u) = diffuse_phi2;

            sp_dolp2_img.at<float>(v,u) = specular_rho2;
            sp_aolp2_img.at<float>(v,u) = specular_phi2;

        }
    }
    cv::imshow("depth1_img",depth1_img);
    cv::imshow("depth2_img",depth2_img);

    showNormal(normal1_img,"map1");
    showNormal(normal2_img,"map2");
        
    showDAoLP(df_dolp1_img,df_aolp1_img,"_DF1");
    showDAoLP(sp_dolp1_img,sp_aolp1_img,"_SP1");

    showDAoLP(df_dolp2_img,df_aolp2_img,"_DF2");
    showDAoLP(sp_dolp2_img,sp_aolp2_img,"_SP2");
    cv::destroyAllWindows();
}

/* =======================================
 * ============== Image IO ===============
 * ======================================= */

void getRealDAoLP(
    const cv::Mat& polar_left_img,
    cv::Mat& RealDoLP_img,
    cv::Mat& RealAoLP_img
){
    RealDoLP_img = cv::Mat::zeros(polar_left_img.rows, polar_left_img.cols, CV_32FC1);    
    RealAoLP_img = cv::Mat::zeros(polar_left_img.rows, polar_left_img.cols, CV_32FC1);    
    
    for (int x = 0; x < polar_left_img.rows; x++){
        for(int y = 0; y < polar_left_img.cols; y++){
            RealDoLP_img.at<float>(x, y) = polar_left_img.at<cv::Vec3w>(x, y)[0] / 10000.0;   
            RealAoLP_img.at<float>(x, y) = polar_left_img.at<cv::Vec3w>(x, y)[1] / 10000.0;                        
        }
    }
}

void outputImage(const cv::Mat& img){        
    for(int i = 0; i < img.rows; i++){
        for(int j = 0; j < img.cols; j++){
            if(img.type() == CV_8UC1)
                cout << img.at<uchar>(i, j) << " ";
            else if(img.type() == CV_16UC1)
                cout << img.at<ushort>(i, j) << " ";
            else if(img.type() == CV_8UC3)
                cout << img.at<cv::Vec3b>(i, j) << " ";
            else if(img.type() == CV_32FC1)
                cout << img.at<float>(i, j) << " ";
            else if(img.type() == CV_32FC3)
                cout << img.at<cv::Vec3f>(i, j) << " ";
            else{
                cout << "datatype not defined" << endl;
                return;
            }
        }
        cout << endl;
    }
}

void saveVisualizedOneChannelImage(
    const cv::Mat img, 
    const std::string path,
    const float min_value,
    const float max_value,
    bool show
){
    if (img.channels()!=1){
        cout << "Image have " << img.channels() << " channels, not one" << endl;
        return;
    }
    cv::Mat visualize_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);    
    for(int i = 0; i < img.rows; i++){
        for(int j = 0; j < img.cols; j++){
            float pix_value;
            if(img.type() == CV_8UC1)
                pix_value = float(img.at<uchar>(i, j));
            else if(img.type() == CV_16UC1)
                pix_value = float(img.at<ushort>(i, j));
            else if(img.type() == CV_32FC1)
                pix_value = img.at<float>(i, j);
            else if (img.type() == CV_32SC1){
                pix_value = float(img.at<int>(i, j));
            }
            else{
                cout << "datatype not defined" << endl;
                return;
            }
            if(pix_value < 0) continue;
            // pix_value /= 2.0;
            visualize_img.at<cv::Vec3b>(i, j) = AoLP2Color((pix_value - min_value) / (max_value - min_value) * M_PI);                              
        }
    }
    cv::imwrite(path, visualize_img); 
    if (show){
        cv::imshow("vis", visualize_img);  
        cv::waitKey();
        cv::destroyAllWindows();  
    }
}

void setNormal(const cv::Mat &norm_gt_img,cv::Mat &normal_img){    
    normal_img = norm_gt_img.clone();
    for (int v = 0; v < norm_gt_img.rows; v++){
        for (int u = 0; u < norm_gt_img.cols; u++) {
            float nx = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[0] - 1;
            float ny = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[1] - 1;
            float nz = 2 * norm_gt_img.at<cv::Vec3f>(v,u)[2] - 1;

            normal_img.at<cv::Vec3f>(v,u)[0] = nz;
            normal_img.at<cv::Vec3f>(v,u)[1] = ny;
            normal_img.at<cv::Vec3f>(v,u)[2] = nx;
        }
    }
}

cv::Mat readLabelMat(std::string FilePath){
    ifstream infile;
    infile.open(FilePath, ios::in);
    if(!infile.is_open()){
        cout << "Cannot find " << FilePath << endl;
        system("pause");
    }

    cv::Mat label;
    string buff;      
    while(getline(infile, buff)){                
        vector<int> label_i;

        char* s_input = (char*)buff.c_str();
        const char* split = " ";
        char* p = strtok(s_input, split);

        int l;
        while(p!=NULL){
            l = atoi(p);
            label_i.push_back(l);
            p = strtok(NULL, split);            
        }        
        
        label.push_back(cv::Mat(1, label_i.size(), cv::DataType<int>::type, label_i.data()));
    }
    // infile.close();            
    return label;
}

std::map<int, vector<float>> readRatio(const string FilePath){    
    ifstream infile;
    infile.open(FilePath, ios::in);
    if(!infile.is_open()){
        cout << "Cannot find " << FilePath << endl;
        system("pause");
    }

    std::map<int, vector<float>> ret;
    string buff;      
    while(getline(infile, buff)){ 
        char* s_input = (char*)buff.c_str();
        const char* split = " ";
        char* p = strtok(s_input, split);

        vector<float> ratio;
        int label;
        bool FIRST = true;
        while(p!=NULL){
            if(FIRST){
                label = atoi(p);
                FIRST = false;
            }
            else{
                ratio.push_back(atof(p));
            }            
            p = strtok(NULL, split);            
        }
        ret[label] = ratio;
    }
    return ret;
}

std::map<int, int> readType(
    const std::string label_type_path     
){
    std::map<int, int> label_type;
    ifstream read_file(label_type_path, ios::in);
    if(!read_file.is_open()){
        std::cout << "Cannot find " << label_type_path << endl;
        system("pause");
    }
    std::string lineStr;
    std::istringstream iss;
    int label;
    int type;

    if(read_file){
        while(getline(read_file, lineStr)){
            iss.clear();
            iss.str(lineStr);
            iss >> label;
            iss >> type;
            label_type[label] = type;
        }
    }
    return label_type;
}



#include "utils.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

float eta = 1.5;
float camera_cx;
float camera_cy;
float camera_fx;
float camera_fy;

cv::Mat convertCoords(const cv::Mat pol){
    cv::Mat Coords = cv::Mat::zeros(pol.rows, pol.cols, CV_32FC2); // at (i, j): (x, y) = (j-cx)/fx, (i-cy)/fy
    for(int i = 0; i < Coords.rows; i++){
        for(int j = 0; j < Coords.cols; j++){
            Coords.at<cv::Vec2f>(i,j)[0] = (j - camera_cx) / camera_fx;
            Coords.at<cv::Vec2f>(i,j)[1] = (i - camera_cy) / camera_fy;
        }
    }
    return Coords;
}

cv::Vec2f solveX(float a,float b,float c){
    if(a!=1){
        b /= a;
        c /= a;
    }
    cv::Vec2f result;
    result[0] = -b / 2.0 + std::sqrt(b * b / 4.0 -  c);
    result[1] = -b / 2.0 - std::sqrt(b * b / 4.0 -  c);
    
    return result;
}


// ========= from supplementary material ============ //
cv::Vec3f diffuseDAoLP2Normal(float dolp, float aolp){
    float theta = diffuseRhoToZenith(dolp, eta);
    float alpha = aolp;
    
    cv::Vec3f normal(cos(alpha)*sin(theta), sin(alpha)*sin(theta), cos(theta));         
    return normal;
}
std::vector<cv::Vec3f> specularDAoLP2Normal(float dolp, float aolp){
    float alpha = aolp + M_PI_2;
    std::vector<float> thetas = specularRhoToZenith(dolp, eta);
    std::vector<cv::Vec3f> normals;

    normals.push_back(cv::Vec3f(cos(alpha)*sin(thetas[0]), sin(alpha)*sin(thetas[0]), cos(thetas[0])));
    normals.push_back(cv::Vec3f(cos(alpha)*sin(thetas[1]), sin(alpha)*sin(thetas[1]), cos(thetas[1])));

    return normals;
}


void getNormalPriors(
    const string pol_path, cv::Mat& diffuse_normal, cv::Mat& specular_normal_1, cv::Mat& specular_normal_2
){
    cv::Mat pol = cv::imread(pol_path, -1);
    cv::Mat DoLP, AoLP;
    getRealDAoLP(pol, DoLP, AoLP);
    
    diffuse_normal = cv::Mat::zeros(pol.rows, pol.cols, CV_32FC3);
    specular_normal_1 = cv::Mat::zeros(pol.rows, pol.cols, CV_32FC3);
    specular_normal_2 = cv::Mat::zeros(pol.rows, pol.cols, CV_32FC3);

    for(int i = 0; i < DoLP.rows; i++){
        for(int j = 0; j < DoLP.cols; j++){
            if (DoLP.at<float>(i,j) >= 1 || DoLP.at<float>(i,j) < 0) continue;            
            // if (pol.at<cv::Vec3w>(i,j) == cv::Vec3f(0,0,0)) continue;
            // cout << DoLP.at<float>(i,j) << " " << AoLP.at<float>(i,j) << endl;

            cv::Vec3f d_normal = diffuseDAoLP2Normal(DoLP.at<float>(i,j),AoLP.at<float>(i,j));            
            vector<cv::Vec3f> s_normal = specularDAoLP2Normal(DoLP.at<float>(i,j),AoLP.at<float>(i,j)); 

            diffuse_normal.at<cv::Vec3f>(i,j) = d_normal;
            specular_normal_1.at<cv::Vec3f>(i,j) = s_normal[0];
            specular_normal_2.at<cv::Vec3f>(i,j) = s_normal[1];                        
            // cout << d_normal.t() << endl << s_normal[0].t() << endl<< s_normal[1].t() << endl << endl;            
            
        }
    }
}

void processPolars(const string dir, const string save_dir){    
    vector<string> filenames = getFileNames(dir, "clpL_", ".png");       
    cout << filenames.size() << endl; 
    for(auto filename : filenames){
        string pol_path = dir + "/" + filename;
        filename.replace(filename.find("png"), 3, "pfm");
        string d_norm_path = save_dir + "/" + filename.replace(filename.find("clpL"), 4, "diff_n");
        string s_norm_path1 = save_dir + "/" + filename.replace(filename.find("diff_n"), 6, "spec_n1");
        string s_norm_path2 = save_dir + "/" + filename.replace(filename.find("n1"), 2, "n2");
        cout << d_norm_path << endl;  
        // cout << s_norm_path1 << endl;    
        // cout << s_norm_path2 << endl;    

        cv::Mat diffuse_normal, specular_normal_1, specular_normal_2;
        getNormalPriors(pol_path, diffuse_normal, specular_normal_1, specular_normal_2);        
        checkAndCreateDir(save_dir);
        savePFM(diffuse_normal, d_norm_path);
        savePFM(specular_normal_1, s_norm_path1);
        savePFM(specular_normal_2, s_norm_path2);
    } 
}

int main(int argc, char **argv){
    // IRS
    // string pol_path = "/mnt/nas_8/datasets/tiancr/b/Render/IRS/Home/ArchVizInterior03Data/lpL_35.png";

    // SPD
    // string pol_path = "/mnt/nas_8/datasets/tiancr/b/SPD/LabOffice/lpL_22.png";

    
    string root, save_root;
    vector<string> subsets;

    root = "/mnt/nas_8/datasets/tiancr/b/Render1/IRS/";
    save_root = "/mnt/nas_8/datasets/tiancr/b/Render_norm1/IRS/";
    // subsets = {"Home", "Office", "Restaurant", "Store"};
    // subsets = {"Restaurant", "Store"};
    camera_fx = 480;
    camera_fy = 480;
    camera_cx = 479.5;
    camera_cy = 269.5;

    string subset = argv[1];
    for(auto filename : getFileNames(root+subset)){
        cout << root+subset+'/'+filename << endl;
        processPolars(root+subset+'/'+filename, save_root+subset+'/'+filename); 
    }
    // for(auto subset:subsets)
    //     for(auto filename : getFileNames(root+subset)){
    //         cout << root+subset+'/'+filename << endl;
    //         processPolars(root+subset+'/'+filename, save_root+subset+'/'+filename); 
    //     }

    // root = "/mnt/nas_8/datasets/tiancr/b/SPD/";
    // subsets = {"Classroom", "Gallery", "LabOffice", "MeetingRoom", "Office", "Outdoor"};
    // camera_fx = 1192.7164;
    // camera_fy = 1192.7108;
    // camera_cx = 664.3392;
    // camera_cy = 482.9869;
    // for(auto subset:subsets)
    //     processPolars(root+subset); 
    
    // visualizeNormal(diffuse_normal, true, "diffuse");
    // visualizeNormal(specular_normal_1, true, "specular1");
    // visualizeNormal(specular_normal_2, true, "specular2");
    

}


#include "polar_processor.h"

using namespace std;
#define DIFFUSE 1
#define SPECULAR 0
void PolarsToStokes(const vector<cv::Mat>& polars_img,vector<cv::Mat>& stokes){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        for(int i =0;i<4;i++){
            cv::Mat polar_gray;
            cv::cvtColor(polars_img[i], polar_gray, cv::COLOR_BGR2GRAY);
            polars.push_back(polar_gray);
        }
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }

    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    stokes.clear();
    for(int i = 0; i<4; i++){
        stokes.push_back(cv::Mat::zeros(rows,cols,CV_16SC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int I_0 = polars[0].at<uchar>(v,u);
            int I_45 = polars[1].at<uchar>(v,u);
            int I_90 = polars[2].at<uchar>(v,u);
            int I_135 = polars[3].at<uchar>(v,u);
            int s0 = I_0 + I_90;
            int s1 = I_0 - I_90;
            int s2 = I_45 - I_135;
            int s3 = 0;
            stokes[0].at<short>(v,u) = s0;
            stokes[1].at<short>(v,u) = s1;
            stokes[2].at<short>(v,u) = s2;
            stokes[3].at<short>(v,u) = s3;
        }
    }
    return;
}

void StokesToDolpAolp(const vector<cv::Mat>& stokes,cv::Mat& dolp_img,cv::Mat& aolp_img,cv::Mat& intensity_img){
    int rows =  stokes[0].rows;
    int cols =  stokes[0].cols;
    dolp_img = cv::Mat::zeros(rows,cols,CV_32FC1);
    aolp_img = cv::Mat::zeros(rows,cols,CV_32FC1);
    intensity_img = cv::Mat::zeros(rows,cols,CV_8UC1);


    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            float s0 = stokes[0].at<short>(v,u);
            float s1 = stokes[1].at<short>(v,u);
            float s2 = stokes[2].at<short>(v,u);
            float s3 = stokes[3].at<short>(v,u);
            if(s0 == 0 && s1 == 0 && s2==0 && s3 == 0){
                continue;
            }
            int  i_mean = round(s0 / 2.0);
            float  rho = sqrt(s1 * s1 + s2 * s2) / s0;
            // float  phi = atan(s2/s1) / 2.0 ;
            float  phi = atan2(s2,s1) / 2.0 ;
            if(phi<0){
                phi += M_PI;
            }
            dolp_img.at<float>(v,u) = rho;
            aolp_img.at<float>(v,u) = phi;
            intensity_img.at<uchar>(v,u) =  i_mean;
        }
    }

    return;
}

void fitPolarNoise(const vector<cv::Mat>& polars_img,cv::Mat& noise_img, int type){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        vector<vector<cv::Mat>> polars3_img;
        polars3_img.resize(3);
        for(int c = 0;c<3;c++){
            polars3_img[c].resize(4);
        }
        vector<cv::Mat> noise3_img;
        noise3_img.resize(3);
        for(int i = 0;i<4;i++){
            vector<cv::Mat> polar3_img;
            cv::split(polars_img[i], polar3_img);
            for(int c = 0;c<3;c++){
                polars3_img[c][i] = polar3_img[c];
            }
        }
        for(int c = 0;c<3;c++){
            fitPolarNoise(polars3_img[c],noise3_img[c],type);
        }
        cv::merge(noise3_img, noise_img);
        return;
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }
    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    noise_img = cv::Mat::zeros(rows,cols,CV_8UC1);
    int max_df = 0;
    cv::Point max_pt;

    int max_n = 0;

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int I_0 = polars[0].at<uchar>(v,u);
            int I_45 = polars[1].at<uchar>(v,u);
            int I_90 = polars[2].at<uchar>(v,u);
            int I_135 = polars[3].at<uchar>(v,u);
            int I_mean = (I_0 + I_45 + I_90 + I_135)/4;
            int I_mean1 =  (I_0 +  I_90 )/2;
            int I_mean2 =  (I_45 +  I_135 )/2;
            int nn;
            if(I_mean == 0){
                continue;
            }
            if(type == 0){
                nn = std::abs(I_mean1 - I_mean2);
            }else if(type == 1){
                nn =  255*std::abs(I_mean1 - I_mean2)/I_mean;
            // }else if(type == 2){
            //     nn =  I_mean1/I_mean*255;
            // }else if(type == 3){
            //     nn =  I_mean2/I_mean*255;
            }
            noise_img.at<uchar>(v,u) = nn;
            if(nn > max_n){
                max_n = nn;
            }
            if(std::abs(I_mean -I_0 )>max_df){
                max_df = std::abs(I_mean -I_0 );
                max_pt =cv::Point(u,v);
            }else if(std::abs(I_mean -I_45 )>max_df){
                max_df = std::abs(I_mean -I_45 );
                max_pt =cv::Point(u,v);
            }else if(std::abs(I_mean -I_90 )>max_df){
                max_df = std::abs(I_mean -I_90 );
                max_pt =cv::Point(u,v);
            }else if(std::abs(I_mean -I_135 )>max_df){
                max_df = std::abs(I_mean -I_135 );
                max_pt =cv::Point(u,v);
            }
            if(0){
                if(u == 310 && v == 346){
                    std::cout<<cv::Point(u,v)<<I_0<<" "<<I_45<<" "<<I_90<<" "<<I_135<<" "<<I_mean<<" "<<I_mean1<<" "<<I_mean2<<" "<<nn<<std::endl;
                }
                if(u == 298 && v == 324){
                    std::cout<<cv::Point(u,v)<<I_0<<" "<<I_45<<" "<<I_90<<" "<<I_135<<" "<<I_mean<<" "<<I_mean1<<" "<<I_mean2<<" "<<nn<<std::endl;
                }
                if(u == 311 && v == 345){
                    std::cout<<cv::Point(u,v)<<I_0<<" "<<I_45<<" "<<I_90<<" "<<I_135<<" "<<I_mean<<" "<<I_mean1<<" "<<I_mean2<<" "<<nn<<std::endl;
                }
                if(u == 500 && v == 200){
                    std::cout<<cv::Point(u,v)<<I_0<<" "<<I_45<<" "<<I_90<<" "<<I_135<<" "<<I_mean<<" "<<I_mean1<<" "<<I_mean2<<" "<<nn<<std::endl;
                }
            }
        }
    }
    // std::cout<<"N"<<type<<" max_df "<<max_df<<std::endl;
    std::cout<<"N"<<type<<" max_n "<<max_n<<std::endl;
}

void PolarsToDolpAolp(const vector<cv::Mat>& polars_img,cv::Mat& dolp_img,cv::Mat& aolp_img){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        for(int i =0;i<4;i++){
            cv::Mat polar_gray;
            cv::cvtColor(polars_img[i], polar_gray, cv::COLOR_BGR2GRAY);
            polars.push_back(polar_gray);
        }
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }

    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    dolp_img = cv::Mat::zeros(rows,cols,CV_32FC1);
    aolp_img = cv::Mat::zeros(rows,cols,CV_32FC1);

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int I_0 = polars[0].at<uchar>(v,u);
            int I_45 = polars[1].at<uchar>(v,u);
            int I_90 = polars[2].at<uchar>(v,u);
            int I_135 = polars[3].at<uchar>(v,u);
            if(I_0 == 0 && I_45 == 0 && I_90==0 && I_135 == 0){
                continue;
            }

            float s0 = 1.0 * (I_0 + I_90);
            float s1 = 1.0 * (I_0 - I_90);
            float s2 = 1.0 * (I_45 - I_135);
            if(s0 == 0){
                continue;
            }
            float  rho = sqrt(s1 * s1 + s2 * s2) / s0;
            float  phi = atan2(s2,s1) / 2.0 ;
            if(phi<0){
                phi += M_PI;
            }
            dolp_img.at<float>(v,u) = rho;
            aolp_img.at<float>(v,u) = phi;
        }
    }
}

void PolarsToIntensity(const vector<cv::Mat>& polars_img,cv::Mat& intensity_img){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        vector<vector<cv::Mat>> polars3_img;
        polars3_img.resize(3);
        for(int c = 0;c<3;c++){
            polars3_img[c].resize(4);
        }
        vector<cv::Mat> intensity3_img;
        intensity3_img.resize(3);
        for(int i = 0;i<4;i++){
            vector<cv::Mat> polar3_img;
            cv::split(polars_img[i], polar3_img);
            for(int c = 0;c<3;c++){
                polars3_img[c][i] = polar3_img[c];
            }
        }
        for(int c = 0;c<3;c++){
            PolarsToIntensity(polars3_img[c],intensity3_img[c]);
        }
        cv::merge(intensity3_img, intensity_img);
        return;
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }

    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    intensity_img = cv::Mat::zeros(rows,cols,CV_8UC1);

    if(1){
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                int I_0 = polars[0].at<uchar>(v,u);
                int I_90 = polars[2].at<uchar>(v,u);
                int I_mean = (I_0 + I_90)/2;
                intensity_img.at<uchar>(v,u) = I_mean;
            }
        }
    }else{
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                int I_0 = polars[0].at<uchar>(v,u);
                int I_45 = polars[1].at<uchar>(v,u);
                int I_90 = polars[2].at<uchar>(v,u);
                int I_135 = polars[3].at<uchar>(v,u);
                int I_mean = (I_0 + I_45 + I_90 + I_135)/4;
                intensity_img.at<uchar>(v,u) = I_mean;
            }
        }
    }
}

void PolarsToS1S2(const vector<cv::Mat>& polars_img,cv::Mat& S1_img,cv::Mat& S2_img){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        vector<vector<cv::Mat>> polars3_img;
        polars3_img.resize(3);
        for(int c = 0;c<3;c++){
            polars3_img[c].resize(4);
        }
        vector<cv::Mat> S1s_img;
        vector<cv::Mat> S2s_img;
        S1s_img.resize(3);
        S2s_img.resize(3);
        for(int i = 0;i<4;i++){
            vector<cv::Mat> polar3_img;
            cv::split(polars_img[i], polar3_img);
            for(int c = 0;c<3;c++){
                polars3_img[c][i] = polar3_img[c];
            }
        }
        for(int c = 0;c<3;c++){
            PolarsToS1S2(polars3_img[c],S1s_img[c],S2s_img[c]);
        }
        cv::merge(S1s_img, S1_img);
        cv::merge(S2s_img, S2_img);
        return;
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }

    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    if(0){
        S1_img = cv::Mat::zeros(rows,cols,CV_16SC1);
        S2_img = cv::Mat::zeros(rows,cols,CV_16SC1);

        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                int I_0 = polars[0].at<uchar>(v,u);
                int I_45 = polars[1].at<uchar>(v,u);
                int I_90 = polars[2].at<uchar>(v,u);
                int I_135 = polars[3].at<uchar>(v,u);
                int s1 = I_0 - I_90;
                int s2 = I_45 - I_135;
                S1_img.at<short>(v,u) = s1;
                S2_img.at<short>(v,u) = s2;
            }
        }
    }else{
        S1_img = cv::Mat::zeros(rows,cols,CV_8UC1);
        S2_img = cv::Mat::zeros(rows,cols,CV_8UC1);

        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                int I_0 = polars[0].at<uchar>(v,u);
                int I_45 = polars[1].at<uchar>(v,u);
                int I_90 = polars[2].at<uchar>(v,u);
                int I_135 = polars[3].at<uchar>(v,u);
                int s1 = I_0 - I_90;
                int s2 = I_45 - I_135;
                S1_img.at<uchar>(v,u) = s1+128;
                S2_img.at<uchar>(v,u) = s2+128;
            }
        }

    }
}

void PolarsToS4(const vector<cv::Mat>& polars_img,cv::Mat& S4_img){
    vector<cv::Mat> polars;
    if(polars_img[0].channels() == 3){
        for(int i =0;i<4;i++){
            cv::Mat polar_gray;
            cv::cvtColor(polars_img[i], polar_gray, cv::COLOR_BGR2GRAY);
            polars.push_back(polar_gray);
        }
    }else{
        for(int i =0;i<4;i++){
            polars.push_back(polars_img[i].clone());
        }
    }

    int rows =  polars[0].rows;
    int cols =  polars[0].cols;
    S4_img = cv::Mat::zeros(rows,cols,CV_8UC3);

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int I_0 = polars[0].at<uchar>(v,u);
            int I_45 = polars[1].at<uchar>(v,u);
            int I_90 = polars[2].at<uchar>(v,u);
            int I_135 = polars[3].at<uchar>(v,u);
            // int s0 = (I_0 + I_90);
            int s0 = (I_0 + I_90 + I_45 + I_135);

            int s1 = (I_0 - I_90);
            int s2 = (I_45 - I_135);
            int s4 = (I_0 + I_90 - I_45 - I_135);

            S4_img.at<cv::Vec3b>(v,u)[0] = s0/4;
            // S4_img.at<cv::Vec3b>(v,u)[0] = s4 + 128;
            S4_img.at<cv::Vec3b>(v,u)[1] = s1 + 128;
            S4_img.at<cv::Vec3b>(v,u)[2] = s2 + 128;
        }
    }
}

void DolpAolpToS4(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,cv::Mat& S4_img){
    cv::Mat intensity;
    if(intensity_img.channels() == 3){
        cv::cvtColor(intensity_img, intensity, cv::COLOR_BGR2GRAY);
    }else{
        intensity = intensity_img.clone();
    }

    int rows =  intensity_img.rows;
    int cols =  intensity_img.cols;
    S4_img = cv::Mat::zeros(rows,cols,CV_8UC3);

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            if(rho == 0.0 && phi == 0){
                continue;
            }
            int  i_mean = intensity.at<uchar>(v,u);

            int s0 = 2* i_mean;
            int s1 = round(2* rho * i_mean * cos(2 * phi));
            int s2 = round(2* rho * i_mean * sin(2 * phi));
            int s3 = 0;

            S4_img.at<cv::Vec3b>(v,u)[0] = s0/2;
            S4_img.at<cv::Vec3b>(v,u)[1] = s1 + 128;
            S4_img.at<cv::Vec3b>(v,u)[2] = s2 + 128;
        }
    }
}

void IntensityToS0(const cv::Mat& intensity_img,cv::Mat& S0){
    if(intensity_img.channels() == 3){
        vector<cv::Mat> S0s_img;
        S0s_img.resize(3);
        vector<cv::Mat> intensity3_img;
        intensity3_img.resize(3);

        cv::split(intensity_img, intensity3_img);

        for(int c = 0;c<3;c++){
            IntensityToS0(intensity3_img[c],S0s_img[c]);
        }
        cv::merge(S0s_img, S0);

        return;
    }

    int rows =  intensity_img.rows;
    int cols =  intensity_img.cols;
    S0  = cv::Mat::zeros(rows,cols,CV_16SC1);

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int  i_mean = intensity_img.at<uchar>(v,u);
            int s0 = 2 * i_mean;
            S0.at<short>(v,u) = 2 * s0;
        }
    }
}

void DolpAolpToPolars(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& polars_img,
                      const cv::Mat& dolp_img_false,const cv::Mat& aolp_img_false)
{
    cout << "DolpAolpToPolars2" << endl;
    if(intensity_img.channels() == 3){
        vector<vector<cv::Mat>> polars3_img;
        polars3_img.resize(4);
        for(int i = 0; i<4; i++){
            polars3_img[i].resize(3);
        }
        vector<cv::Mat> intensity3_img;
        intensity3_img.resize(3);

        cv::split(intensity_img, intensity3_img);

        for(int c = 0;c<3;c++){
            vector<cv::Mat> polars;
            DolpAolpToPolars(dolp_img,aolp_img,intensity3_img[c],polars,dolp_img_false,aolp_img_false);
            for(int i = 0; i<4; i++){
                polars3_img[i][c] = polars[i];
            }
        }

        polars_img.resize(4);
        for(int i = 0; i<4; i++){
            cv::merge(polars3_img[i], polars_img[i]);
        }
        return;
    }

    int rows =  intensity_img.rows;
    int cols =  intensity_img.cols;
    polars_img.clear();
    for(int i = 0; i<4; i++){
        polars_img.push_back(cv::Mat::zeros(rows,cols,CV_8UC1));
    }

    int count = 0;
    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            int i_mean = intensity_img.at<uchar>(v,u) / 2;
            int I_0 = round( i_mean + rho * i_mean * cos(2 * phi));
            int I_45 = round( i_mean + rho * i_mean * sin(2 * phi));
            int I_90 = round( i_mean - rho * i_mean * cos(2 * phi));
            int I_135 = round( i_mean - rho * i_mean * sin(2 * phi));

            // if (I_0 > 255 || I_45 > 255 || I_90 > 255 || I_135 > 255){
            //     rho = dolp_img_false.at<float>(v,u);
            //     phi = aolp_img_false.at<float>(v,u);
            //     i_mean = intensity_img.at<uchar>(v,u) / 2;
            //     I_0 = round( i_mean + rho * i_mean * cos(2 * phi));
            //     I_45 = round( i_mean + rho * i_mean * sin(2 * phi));
            //     I_90 = round( i_mean - rho * i_mean * cos(2 * phi));
            //     I_135 = round( i_mean - rho * i_mean * sin(2 * phi));
            //     count ++;
            // }


            polars_img[0].at<uchar>(v,u) = I_0;
            polars_img[1].at<uchar>(v,u) = I_45;
            polars_img[2].at<uchar>(v,u) = I_90;
            polars_img[3].at<uchar>(v,u) = I_135;
        }
    }
    cout << count << ' ' << endl;
}

void DolpAolpToPolars(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& polars_img){
    if(intensity_img.channels() == 3){
        vector<vector<cv::Mat>> polars3_img;
        polars3_img.resize(4);
        for(int i = 0; i<4; i++){
            polars3_img[i].resize(3);
        }
        vector<cv::Mat> intensity3_img;
        intensity3_img.resize(3);

        cv::split(intensity_img, intensity3_img);

        for(int c = 0;c<3;c++){
            vector<cv::Mat> polars;
            DolpAolpToPolars(dolp_img,aolp_img,intensity3_img[c],polars);
            for(int i = 0; i<4; i++){
                polars3_img[i][c] = polars[i];
            }
        }

        polars_img.resize(4);
        for(int i = 0; i<4; i++){
            cv::merge(polars3_img[i], polars_img[i]);
        }
        return;
    }

    int rows =  intensity_img.rows;
    int cols =  intensity_img.cols;
    polars_img.clear();
    for(int i = 0; i<4; i++){
        polars_img.push_back(cv::Mat::zeros(rows,cols,CV_8UC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            int i_mean = intensity_img.at<uchar>(v,u);
            int I_0 = round( i_mean + rho * i_mean * cos(2 * phi));
            int I_45 = round( i_mean + rho * i_mean * sin(2 * phi));
            int I_90 = round( i_mean - rho * i_mean * cos(2 * phi));
            int I_135 = round( i_mean - rho * i_mean * sin(2 * phi));

            if (I_0 > 255){
                I_0 = 255;
            }

            if (I_45 > 255){
                I_45 = 255;
            }
            if (I_90 > 255) {
                I_90 = 255;
            }
            if (I_135 > 255) {
                I_135 = 255;
            }

            polars_img[0].at<uchar>(v,u) = I_0;
            polars_img[1].at<uchar>(v,u) = I_45;
            polars_img[2].at<uchar>(v,u) = I_90;
            polars_img[3].at<uchar>(v,u) = I_135;
        }
    }
}

void DolpAolpToStokes(const cv::Mat& dolp_img,const cv::Mat& aolp_img,const cv::Mat& intensity_img,vector<cv::Mat>& stokes){
    cv::Mat intensity;

    if(intensity_img.channels() == 3){
        cv::cvtColor(intensity_img, intensity, cv::COLOR_BGR2GRAY);
    }else{
        intensity = intensity_img.clone();
    }

    int rows =  intensity_img.rows;
    int cols =  intensity_img.cols;
    stokes.clear();
    for(int i = 0; i<4; i++){
        stokes.push_back(cv::Mat::zeros(rows,cols,CV_16SC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            int  i_mean = intensity.at<uchar>(v,u);

            int s0 = 2* i_mean;
            int s1 = round(2* rho * i_mean * cos(2 * phi));
            int s2 = round(2* rho * i_mean * sin(2 * phi));
            int s3 = 0;
            stokes[0].at<short>(v,u) = s0;
            stokes[1].at<short>(v,u) = s1;
            stokes[2].at<short>(v,u) = s2;
            stokes[3].at<short>(v,u) = s3;
        }
    }
}

void DolpAolpToNormal(const cv::Mat& dolp_img,const cv::Mat& aolp_img,int ref_type, vector<cv::Mat>& normal_img){
    cv::Mat zenith_img;
    cv::Mat azimuth_img;
    DolpAolpToZenithAzimuth(dolp_img,aolp_img,ref_type,zenith_img,azimuth_img);

    int rows =  dolp_img.rows;
    int cols =  dolp_img.cols;

    normal_img.clear();
    for(int i = 0; i< zenith_img.channels();i++){
        for(int j = 0; j< azimuth_img.channels();j++){
            normal_img.push_back(cv::Mat::zeros(rows,cols,CV_32FC3));
        }
    }

    if(ref_type == DIFFUSE){
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                for(int k = 0; k<2; k++){
                    float zenith = zenith_img.at<float>(v,u);
                    float azimuth = azimuth_img.at<cv::Vec2f>(v,u)[k];
                    float nx = cos(azimuth)*sin(zenith);
                    float ny = -sin(azimuth)*sin(zenith);
                    float nz = -cos(zenith);
                    normal_img[k].at<cv::Vec3f>(v,u)[0] = nx;
                    normal_img[k].at<cv::Vec3f>(v,u)[1] = ny;
                    normal_img[k].at<cv::Vec3f>(v,u)[2] = nz;
                }
            }
        }
    }

    if(ref_type == SPECULAR){
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                for(int i = 0; i<2; i++){
                    for(int j = 0; j<2; j++){
                        int k = 2*i + j;
                        float zenith = zenith_img.at<cv::Vec2f>(v,u)[i];
                        float azimuth = azimuth_img.at<cv::Vec2f>(v,u)[j];
                        float nx = cos(azimuth)*sin(zenith);
                        float ny = -sin(azimuth)*sin(zenith);
                        float nz = -cos(zenith);
                        normal_img[k].at<cv::Vec3f>(v,u)[0] = nx;
                        normal_img[k].at<cv::Vec3f>(v,u)[1] = ny;
                        normal_img[k].at<cv::Vec3f>(v,u)[2] = nz;
                    }
                }
            }
        }
    }
    std::cout<<zenith_img.at<float>(200,100)<<std::endl;
    std::cout<<azimuth_img.at<float>(200,100)<<std::endl;
    std::cout<<zenith_img.at<float>(100,200)<<std::endl;
    std::cout<<azimuth_img.at<float>(100,200)<<std::endl;
    std::cout<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(100,200)[0]<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(100,200)[1]<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(100,200)[2]<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(200,100)[0]<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(200,100)[1]<<std::endl;
    std::cout<<normal_img[0].at<cv::Vec3f>(200,100)[2]<<std::endl;
}

void DolpAolpToZenithAzimuth(const cv::Mat& dolp_img,const cv::Mat& aolp_img,int ref_type, cv::Mat& zenith_img,cv::Mat& azimuth_img){
    DolpToZenith(dolp_img, ref_type, zenith_img);
    AolpToAzimuth(aolp_img, ref_type, azimuth_img);
}

void DolpToZenith(const cv::Mat& dolp_img,int ref_type, cv::Mat& zenith_img){
    double eta = 1.5;
    int rows =  dolp_img.rows;
    int cols =  dolp_img.cols;

    if(ref_type == DIFFUSE){
        zenith_img = cv::Mat::zeros(rows,cols,CV_32FC1);

        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                float rho = dolp_img.at<float>(v,u);
                float zenith = diffuseRhoToZenith(rho,eta);
                zenith_img.at<float>(v,u) = zenith;
            }
        }
    }
    if(ref_type == SPECULAR){
        zenith_img = cv::Mat::zeros(rows,cols,CV_32FC2);
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                float rho = dolp_img.at<float>(v,u);
                vector<float>  zeniths = specularRhoToZenith(rho,eta);
                zenith_img.at<cv::Vec2f>(v,u)[0] = zeniths[0];
                zenith_img.at<cv::Vec2f>(v,u)[1] = zeniths[1];
            }
        }
    }
}

void AolpToAzimuth(const cv::Mat& aolp_img,int ref_type, cv::Mat& azimuth_img){
    int rows =  aolp_img.rows;
    int cols =  aolp_img.cols;

    azimuth_img = cv::Mat::zeros(rows,cols,CV_32FC2);

    if(ref_type == DIFFUSE){
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                float phi = aolp_img.at<float>(v,u);
                vector<float>  azimuths = diffusePhiToAzimuth(phi);
                azimuth_img.at<cv::Vec2f>(v,u)[0] = azimuths[0];
                azimuth_img.at<cv::Vec2f>(v,u)[1] = azimuths[1];
            }
        }
    }
    if(ref_type == SPECULAR){
        for (int v = 0; v < rows; v++){
            for (int u = 0; u < cols; u++) {
                float phi = aolp_img.at<float>(v,u);
                vector<float>  azimuths = specularPhiToAzimuth(phi);
                azimuth_img.at<cv::Vec2f>(v,u)[0] = azimuths[0];
                azimuth_img.at<cv::Vec2f>(v,u)[1] = azimuths[1];
            }
        }
    }
}

void StokesVisToRaw(const vector<cv::Mat>& vis_stokes,vector<cv::Mat>& stokes){
    int rows =  vis_stokes[0].rows;
    int cols =  vis_stokes[1].cols;

    vector<cv::Mat> gray_stokes;
    for(int i =0;i<4;i++){
        gray_stokes.push_back(cv::Mat::zeros(rows,cols,CV_8UC1));
    }

    for(int i =0;i<4;i++){
        cv::cvtColor(vis_stokes[i], gray_stokes[i], cv::COLOR_BGR2GRAY);
    }

    stokes.clear();
    for(int i =0;i<4;i++){
        stokes.push_back(cv::Mat::zeros(rows,cols,CV_16SC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int vis_s0 = gray_stokes[0].at<uchar>(v,u);
            int vis_s1 = gray_stokes[1].at<uchar>(v,u);
            int vis_s2 = gray_stokes[2].at<uchar>(v,u);
            int vis_s3 = gray_stokes[3].at<uchar>(v,u);
            int s0 = vis_s0 * 2;
            int s1 = vis_s1 - 128;
            int s2 = vis_s2 - 128;
            int s3 = 0;

            stokes[0].at<short>(v,u) = s0;
            stokes[1].at<short>(v,u) = s1;
            stokes[2].at<short>(v,u) = s2;
            stokes[3].at<short>(v,u) = s3;
        }
    }
}

void DolpAolpVisToRaw(vector<cv::Mat>& stokes){

}

void StokesToPolars(const vector<cv::Mat>& stokes,vector<cv::Mat>& polars){
    int rows =  stokes[0].rows;
    int cols =  stokes[0].cols;
    polars.clear();
    for(int i = 0; i<4; i++){
        polars.push_back(cv::Mat::zeros(rows,cols,CV_8UC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int s0 = stokes[0].at<short>(v,u);
            int s1 = stokes[1].at<short>(v,u);
            int s2 = stokes[2].at<short>(v,u);

            int I_0 = (s0 + s1)/2;
            int I_45 = (s0 + s2)/2;
            int I_90 = (s0 - s1)/2;
            int I_135 = (s0 - s2)/2;

            polars[0].at<uchar>(v,u) = I_0;
            polars[1].at<uchar>(v,u) = I_45;
            polars[2].at<uchar>(v,u) = I_90;
            polars[3].at<uchar>(v,u) = I_135;
        }
    }
}

cv::Vec3b DAoLP_2Color(float dolpValue,float aolpValue){
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

cv::Vec3b DAoLP2Color2(float dolpValue,float aolpValue){
        aolpValue = aolpValue / M_PI * 180;
        dolpValue = dolpValue * 255.0;
        if(aolpValue<0){
            aolpValue += 180;
        }
        double hue = (aolpValue)*2.0;
        double saturation = dolpValue / 255.0;
        int value = 255*saturation;

        int c = round(value);
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

cv::Vec3b AoLP_2Color(float aolpValue){
        aolpValue = aolpValue / M_PI * 180;
        if(aolpValue<0){
            aolpValue += 180;
        }

        double hue = (aolpValue)*2.0;
        int value = 255;
        int c = value;
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

cv::Vec3b AoLPHalf2Color(float aolpValue){
        if(aolpValue >M_PI/2){
            aolpValue -= M_PI/2;
        }
        aolpValue = aolpValue / M_PI * 180;
        if(aolpValue<0){
            aolpValue += 180;
        }

        double hue = (aolpValue)*4.0;
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

cv::Vec3b DoLP2Color(float dolpValue){
        double hue = dolpValue * 360.0;
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

char DoLP_2Gray(float dolpValue){
    // int gray_tmp =  dolpValue;
    int gray_tmp =  round( dolpValue * 255);
    char res = gray_tmp;
    return res;
}

cv::Vec3b Norm2Color(cv::Vec3f normValue){
    float nx = normValue[0];
    float ny = normValue[1];
    float nz = normValue[2];
    int red = round(255 * (nx + 1.0)/ 2.0);
    int green = round(255 * (ny + 1.0)/ 2.0);
    int blue = round(255 * (nz + 1.0)/ 2.0);
    return cv::Vec3b(red,green,blue);
}

void show_ColorMap(){
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
    if(0){
      cv::imwrite("/home/tcr/DataSets/Polar/output/depth1_img.png",depth1_img);
      cv::imwrite("/home/tcr/DataSets/Polar/output/depth2_img.png",depth2_img);
    }

    showNormal(normal1_img,"map1");
    showNormal(normal2_img,"map2");

    showDolpAolp(df_dolp1_img,df_aolp1_img,"_DF1");
    showDolpAolp(sp_dolp1_img,sp_aolp1_img,"_SP1");

    showDolpAolp(df_dolp2_img,df_aolp2_img,"_DF2");
    showDolpAolp(sp_dolp2_img,sp_aolp2_img,"_SP2");
}

void showDolpAolp(const cv::Mat& dolp_img,const cv::Mat& aolp_img,std::string suffix){
    std::cout<<"# showDolpAolp"<<std::endl;
    cv::Mat dop_vis_img = cv::Mat::zeros(dolp_img.rows,dolp_img.cols,CV_8UC1);
    // cv::Mat dop_vis_img = cv::Mat::zeros(dolp_img.rows,dolp_img.cols,CV_8UC3);
    cv::Mat aop_vis_img = cv::Mat::zeros(aolp_img.rows,aolp_img.cols,CV_8UC3);

    cv::Mat lp_vis_img = cv::Mat::zeros(dolp_img.rows,dolp_img.cols,CV_8UC3);

    for (int v = 1; v < dolp_img.rows - 1; v++){
        for (int u = 1; u < dolp_img.cols - 1; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            dop_vis_img.at<uchar>(v,u) = DoLP_2Gray(rho);
            // dop_vis_img.at<cv::Vec3b>(v,u) = DoLP2Color(rho);
            aop_vis_img.at<cv::Vec3b>(v,u) = AoLP_2Color(phi);
            // aop_vis_img.at<cv::Vec3b>(v,u) = AoLPHalf2Color(phi);
            lp_vis_img.at<cv::Vec3b>(v,u) = DAoLP_2Color(rho,phi);
            // lp_vis_img.at<cv::Vec3b>(v,u) = DAoLP2Color2(rho,phi);
        }
    }

    std::cout<<"dolp_img size : "<<dolp_img.size()<<std::endl;
    std::cout<<"aolp_img size : "<<aolp_img.size()<<std::endl;

    cv::imshow("dop_vis_img"+suffix,dop_vis_img);
    cv::imshow("aop_vis_img"+suffix,aop_vis_img);
    cv::imshow("lp_vis_img"+suffix,lp_vis_img);

    cv::waitKey();
    if(1){
      cv::imwrite("/home/tcr/DataSets/Polar/output/dop_vis_img"+suffix+".png",dop_vis_img);
      cv::imwrite("/home/tcr/DataSets/Polar/output/aop_vis_img"+suffix+".png",aop_vis_img);
      cv::imwrite("/home/tcr/DataSets/Polar/output/lp_vis_img"+suffix+".png",lp_vis_img);
    }
    return;
}

void visualizeDolpAolp(const cv::Mat& dolp_img,const cv::Mat& aolp_img,cv::Mat &dop_vis_img,cv::Mat &aop_vis_img){
    dop_vis_img = cv::Mat::zeros(dolp_img.rows,dolp_img.cols,CV_8UC1);
    aop_vis_img = cv::Mat::zeros(aolp_img.rows,aolp_img.cols,CV_8UC3);
    for (int v = 1; v < dolp_img.rows - 1; v++){
        for (int u = 1; u < dolp_img.cols - 1; u++) {
            float rho = dolp_img.at<float>(v,u);
            float phi = aolp_img.at<float>(v,u);
            if(rho == 0 && phi == 0){
                continue;
            }
            dop_vis_img.at<uchar>(v,u) = DoLP_2Gray(rho);
            aop_vis_img.at<cv::Vec3b>(v,u) = AoLP_2Color(phi);
        }
    }
    return;
}

void showStokes(const vector<cv::Mat>& stokes,std::string suffix){
    std::cout<<"# showStokes"<<std::endl;
    vector<std::string> file_names;
    file_names.push_back("s0");
    file_names.push_back("s1");
    file_names.push_back("s2");
    file_names.push_back("s3");

    int rows =  stokes[0].rows;
    int cols =  stokes[0].cols;

    vector<cv::Mat> vis_stokes;
    for(int i =0;i<4;i++){
        vis_stokes.push_back(cv::Mat::zeros(rows,cols,CV_8UC1));
    }

    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++) {
            int s0 = stokes[0].at<short>(v,u);
            int s1 = stokes[1].at<short>(v,u);
            int s2 = stokes[2].at<short>(v,u);
            int s3 = stokes[3].at<short>(v,u);
            vis_stokes[0].at<uchar>(v,u) = s0/2;
            vis_stokes[1].at<uchar>(v,u) = s1 + 128;
            vis_stokes[2].at<uchar>(v,u) = s2 + 128;
        }
    }

    for(int i =0;i<stokes.size();i++){
         cv::imshow(file_names[i]+"_"+suffix,vis_stokes[i]);
    }
    cv::waitKey();
}

void showPolars(const vector<cv::Mat>& polars_img,std::string suffix){
    std::cout<<"# showPolars"<<std::endl;
    vector<std::string> file_names;
    file_names.push_back("polar0");
    file_names.push_back("polar45");
    file_names.push_back("polar90");
    file_names.push_back("polar135");

    for(int i =0;i<4;i++){
         cv::imshow(file_names[i]+"_"+suffix,polars_img[i]);
    }
    cv::waitKey();
}

void showIntensity(const cv::Mat& intensity_img,std::string suffix){
    std::cout<<"# showIntensity"<<std::endl;
    cv::imshow("intensity_img"+suffix,intensity_img);
    cv::waitKey();
}

void showNormal(const cv::Mat& normal_img,std::string suffix){
    std::cout<<"# showNormal"<<std::endl;
    cv::Mat norm_vis_img = cv::Mat::zeros(normal_img.rows,normal_img.cols,CV_8UC3);

    for (int v = 1; v < normal_img.rows - 1; v++){
        for (int u = 1; u < normal_img.cols - 1; u++) {
            cv::Vec3f norm = normal_img.at<cv::Vec3f>(v,u);
            norm_vis_img.at<cv::Vec3b>(v,u) = Norm2Color(norm);
        }
    }
    cv::imshow("vis_norm_"+suffix,norm_vis_img);
    cv::waitKey();
    if(1){
      cv::imwrite("/home/tcr/DataSets/Polar/output/vis_norm_"+suffix+".png",norm_vis_img);
    }
}

void savePfmStokes(const cv::Mat& S1_img,const cv::Mat& S2_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix){
    std::string suffix = ".pfm";

    std::string s1_pfm= syn_dir + "/" + reflect_fix +"s1" + fix + view_str + suffix;
    std::string s2_pfm = syn_dir + "/" + reflect_fix +"s2" + fix + view_str + suffix;

    std::cout<<"s1_pfm:    "<<s1_pfm<<std::endl;
    std::cout<<"s2_pfm:    "<<s2_pfm<<std::endl;


    // return;

    savePFM(S1_img,s1_pfm);
    savePFM(S2_img,s2_pfm);
}

void savePngStokes(const cv::Mat& S1_img,const cv::Mat& S2_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix){
    std::string suffix = ".png";

    std::string s1_png= syn_dir + "/" + reflect_fix +"s1" + fix + view_str + suffix;
    std::string s2_png = syn_dir + "/" + reflect_fix +"s2" + fix + view_str + suffix;

    std::cout<<"s1_png:    "<<s1_png<<std::endl;
    std::cout<<"s2_png:    "<<s2_png<<std::endl;
    // return;

    cv::imwrite(s1_png,S1_img);
    cv::imwrite(s2_png,S2_img);
}

void savePngS4(const cv::Mat& S4_img,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix){
    std::string suffix = ".png";

    std::string s4_png= syn_dir + "/" + reflect_fix +"s4" + fix + view_str + suffix;

    std::cout<<"s4_png:    "<<s4_png<<std::endl;

    cv::imwrite(s4_png,S4_img);
}

void savePngPolars(const vector<cv::Mat>& polars,std::string reflect_fix,std::string syn_dir,std::string view_str,std::string fix){
    std::string suffix = ".png";

    std::string p0_png = syn_dir + "/" + reflect_fix +"p0" + fix + view_str + suffix;
    std::string p1_png= syn_dir + "/" + reflect_fix +"p1" + fix + view_str + suffix;
    std::string p2_png = syn_dir + "/" + reflect_fix +"p2" + fix + view_str + suffix;
    std::string p3_png = syn_dir + "/" + reflect_fix +"p3" + fix + view_str + suffix;


    std::cout<<"p1_png:    "<<p1_png<<std::endl;
    std::cout<<"p2_png:    "<<p2_png<<std::endl;
    // return;

    cv::imwrite(p0_png,polars[1]);
    cv::imwrite(p1_png,polars[2]);
    cv::imwrite(p2_png,polars[1]);
    cv::imwrite(p3_png,polars[2]);

}

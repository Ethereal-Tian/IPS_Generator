#include  "DolpAolpToZenithAzimuth.h"

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
     

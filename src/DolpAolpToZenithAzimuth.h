#include <fstream>
#include <sstream>
#include <iostream>

#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <math.h>

// quartic equation solver

std::vector<float> solveQuartic(float a,float b,float c,float d,float e);

float solveQuadratic(float a,float b,float c);

float diffuseZenithToRho(float zenith,float eta);

float specularZenithToRho(float zenith,float eta);

float diffuseRhoToZenith(float rho,float eta);

std::vector<float>  specularRhoToZenith(float rho,float eta);

float diffuseAzimuthToPhi(float azimuth);

float specularAzimuthToPhi(float azimuth);

std::vector<float> diffusePhiToAzimuth(float phi);

std::vector<float> specularPhiToAzimuth(float phi);

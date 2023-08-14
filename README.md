### IPS Generator
**IPS Generator** is a toolkit for generating the synthetic polarimetric stereo dataset named as
IPS dataset in our paper ( **DPS-Net: Deep Polarimetric Stereo Depth Estimation** ).

## 1. Introduction

We present **IPS Generator** to synthesize the polarimetric data of IPS from the accurate normal map provided by the [IRS](https://github.com/HKBU-HPML/IRS.git). The surface normal is decomposed as the azimuth angle and the zenith angle to calculate the AoLP image and the DoLP image dominated by the specular reflection or diffuse reflection. The dominant reflection types are determined by the segmentation results of the [SeMask-Segmentation](https://github.com/Picsart-AI-Research/SeMask-Segmentation.git), which was SOTA during paper submission. [SAM](https://github.com/facebookresearch/segment-anything.git) is recommended for better segmentation quality. The synthetic DoLP and AoLP maps in the IPS dataset are finally obtained after adding Gaussian noise into the polarized images.

## 2. Installation

### 2.1. Python Environment

For the python environment, please refer to  [SeMask-Segmentation](https://github.com/Picsart-AI-Research/SeMask-Segmentation.git).

Then, init SeMask submodule with
```bash
git submodule --init --recursive
```

Next, the pre-trained model [semask_large_mask2former_ade20k.pth](https://drive.google.com/file/d/1hN1I4Wv7_1FCPOsfA-5PELn6Xn3b_R8a/view) for segmentation can be download, following  [SAM](https://github.com/facebookresearch/segment-anything.git). 

### 2.2. C++ Environment
First, please install the following dependencies
- Eigen
- OpenCV
- PCL

Then, the cpp code can be compiled
```bash
mkdir build
cd build
cmake ..
make -j
```
## 3. Usage
First, run following python scripts to segment the  RGB images for generating the dominant reflection type
```bash
python sem_seg.py
```
Then, synthesis DoLP and AoLP can be produced by executing the executable file.
```bash
./rebuild_by_ratio_distribution
```

## 4. Real Data
The real polarimetric dataset is captured as well. The RPS dataset utilized in **DPS-Net** can be download from [Google Drive](https://drive.google.com/drive/folders/1R3ODf5ykpVWOP8Xtj092YbwGG3DYC6Dt?usp=sharing).
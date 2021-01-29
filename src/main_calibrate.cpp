//author email: vision3d@yeah.net
//感谢微信公众号：3D视觉工坊 及 计算机视觉工坊

#include "calibCoreAlgorithm.h"

#include <math.h>

#include <iostream>

using namespace cv;
using namespace std;

#define fisheye_calib_GUESS

//#define pinhole_calib_GUESS

//#define fisheye_calib

//#define pinhole_calib


int main( int argc, char** argv )
{

#ifdef fisheye_calib_GUESS
    //step-1:left cam calib
    calibCoreAlgorithm camL;
    camL._camera._cameraL=true;
    camL._configPath="../config/config.yaml";

    camL.doSingleCalibrateFishEyeROI();

    //step-2:right cam calib
    calibCoreAlgorithm camR;
    camR._configPath=camL._configPath;
    camR._camera._cameraR=true;

    camR.doSingleCalibrateFishEyeROI();

    std::cout<<"begin to stereocalib ..."<<endl;
    //stereoCalib
    RT stero_RT;
    if(!calibCoreAlgorithm::StereoCalibFishEyeROI(camL,camR,stero_RT))
    {
        return  false;
    }

#endif

#ifdef fisheye_calib

    //step-1:left cam calib
    calibCoreAlgorithm camLeft;
    camLeft._camera._cameraL=true;
    camLeft._configPath="../config/config.yaml";

    camLeft.doSingleCalibrateFishEye();

    //step-2:right cam calib
    calibCoreAlgorithm camRight;
    camRight._configPath=camLeft._configPath;
    camRight._camera._cameraR=true;

    camRight.doSingleCalibrateFishEye();

    std::cout<<"begin to stereocalib ..."<<endl;
    //stereoCalib
    RT stero_RT;
    if(!calibCoreAlgorithm::StereoCalibFishEye(camLeft,camRight,stero_RT))
    {
        return  false;
    }

#endif

#ifdef pinhole_calib

    //step-1:left cam calib
    calibCoreAlgorithm camLeft_pinhole;
    camLeft_pinhole._camera._cameraL=true;
    camLeft_pinhole._configPath="../config/config.yaml";

    camLeft_pinhole.doSingleCalibratePinHole();

    //step-2:right cam calib
    calibCoreAlgorithm camRight_pinhole;
    camRight_pinhole._configPath=camLeft_pinhole._configPath;
    camRight_pinhole._camera._cameraR=true;

    camRight_pinhole.doSingleCalibratePinHole();

    std::cout<<"begin to stereocalib ..."<<endl;
    //stereoCalib
    RT stero_RT_pinhole;
    if(!calibCoreAlgorithm::StereoCalibPinHole(camLeft_pinhole,camRight_pinhole,stero_RT_pinhole))
    {
        return  false;
    }

#endif

#ifdef pinhole_calib_GUESS

    //step-1:left cam calib
    calibCoreAlgorithm camLeft_pinhole_UseGuess;
    camLeft_pinhole_UseGuess._camera._cameraL=true;
    camLeft_pinhole_UseGuess._configPath="../config/config.yaml";

    camLeft_pinhole_UseGuess.doSingleCalibratePinHole_UseGuess();

    //step-2:right cam calib
    calibCoreAlgorithm camRight_pinhole_UseGuess;
    camRight_pinhole_UseGuess._configPath=camLeft_pinhole_UseGuess._configPath;
    camRight_pinhole_UseGuess._camera._cameraR=true;

    camRight_pinhole_UseGuess.doSingleCalibratePinHole_UseGuess();

    std::cout<<"begin to stereocalib ..."<<endl;

    //stereoCalib
    if(!calibCoreAlgorithm::StereoCalibPinHole_UseGuess(camLeft_pinhole_UseGuess,camRight_pinhole_UseGuess))
    {
        return  false;
    }

#endif
    return true;
}

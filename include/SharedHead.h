#pragma once

#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>

struct calibCameraLR
{
    calibCameraLR():_cameraL(false),_cameraR(false)
    {

    }
public:
    bool _cameraL;
    bool _cameraR;
};



struct RT
{
    cv::Mat _R; //3x1
    cv::Mat _T; //3x1
	double _stero_rms;
	double _length_T;
};

//pinhole camera para
class PinholeCamPara
{
public:
    PinholeCamPara()
    {
        _ReprojectionError[0]=0; _ReprojectionError[1]=0;
		_cameraMatrix = cv::Mat::zeros(3,3,CV_64FC1);
		_distortMatrix = cv::Mat::zeros(1, 4, CV_64FC1);
    }

public:
    cv::Mat _cameraMatrix; 

public:
    cv::Mat _distortMatrix;

    std::vector<RT> _imgRTVec;   //calib board 
    cv::Mat _parallelCamR;       //parallel view R
    double _ReprojectionError[2]; //ReprojectionError X and Y  important
    std::vector<double> _reprojectNormErr;
    double _totalReproNormErr;

};

//fisheye kannala brandt
class FishEyeCamPara
{
public:
    FishEyeCamPara():_totalReproNormErr(0)
    {
        _ReprojectionError[0]=0;_ReprojectionError[1]=0;
        _cameraMatrix = cv::Mat::zeros(3,3,CV_64FC1);
        _DistortCoeff=cv::Mat::zeros(4,1,CV_64FC1);
    }


public:
    cv::Mat _cameraMatrix; //cam intrinsic matrix
    cv::Mat _DistortCoeff; //distortCoeffs
    std::vector<RT> _imgRTVec;   //calib board

public:
    double _ReprojectionError[2]; //ReprojectionError X and Y  important
    std::vector<double> _reprojectNormErr;
    double _totalReproNormErr;
};



class CalibrationData
{
    public:
    CalibrationData():_cornerRows(11),
    _cornerCols(4),
    _image_width(0),
    _image_height(0),
    _boardSize(cv::Size(_cornerRows,_cornerCols)),
            _imageSize(cv::Size(_image_width,_image_height))
    {

    }
public:
    int _cornerRows;
    int _cornerCols;
    int _image_width;
    int _image_height;

public:
    cv::Size _boardSize;
    cv::Size _imageSize;
    std::vector<cv::Mat> _tvecsMat;  //each image translation
    std::vector<cv::Mat> _rvecsMat;  //each image rotation
	std::vector<std::vector<cv::Point2f>> _image_corner_pnt2fs_vec;//corner detected
    std::vector<std::vector<cv::Point3f>> _plane3fPntsVec; //calib board 3D corner
	std::vector<std::vector<cv::Point2f>> _plane2fPntsVec; //calib board 2D corner

	std::vector<std::vector<cv::Point2d>> _image_corner_pnt2ds_vec;//corner detected for fisheye calib
	std::vector<std::vector<cv::Point2d>> _plane2dPntsVec; //calib board 2D corner for fisheye calib
	std::vector<std::vector<cv::Point3d>> _plane3dPntsVec; //calib board 3D corner
};








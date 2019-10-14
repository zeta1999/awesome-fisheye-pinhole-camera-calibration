#ifndef INTRINSIC_H_INCLUDED
#define INTRINSIC_H_INCLUDED


#include <fstream>
#include <sstream>
#include <iterator>

#include "include/SharedHead.h"

using namespace cv;
using namespace std;


class calibCoreAlgorithm {

public:
    calibCoreAlgorithm() : _image_count(0)
                           {
        _pair_corner_pnts_image_vec.clear();
		_pair_corner_pnt2ds_image_vec.clear();
    }

    ~calibCoreAlgorithm() {

    }

public:
    bool doSingleCalibratePinHole();

    bool doSingleCalibratePinHole_UseGuess();

    bool doSingleCalibrateFishEye();

    bool doSingleCalibrateFishEyeROI();

    bool ZhangCalibrationMethodPinHole(CalibrationData &cData, PinholeCamPara &campara);

    bool ZhangCalibrationMethodPinHole_UseGuess(CalibrationData &cData, PinholeCamPara &campara);

    bool ZhangCalibrationKannalaBrandt(CalibrationData &cData,FishEyeCamPara &camPara);

    bool ZhangCalibrationKannalaBrandtROI(CalibrationData &cData);

    static bool StereoCalibPinHole(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR,RT &stereo_RT);

    static bool StereoCalibPinHole_UseGuess(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR);

    static bool StereoCalibFishEye(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR,RT &stereo_RT);

    static bool StereoCalibFishEyeROI(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR,RT &stereo_RT);

    bool detectCornersPinhole(const cv::Mat imageInput, cv::Size board_size, std::vector<cv::Point2f> &image_points);

	bool detectCornersFishEye(const cv::Mat imageInput, cv::Size board_size, std::vector<cv::Point2d> &image_points);

    bool detectCornersFishEyeROI(const cv::Mat imageInput,cv::Rect maskRoi, cv::Size board_size, std::vector<cv::Point2d> &image_points);

    void createROI(const cv::Mat image,int num_roi, std::vector<cv::Rect> &MaskRoiVec);

    void createROI_test(const cv::Mat image,int num_roi, std::vector<cv::Rect> &MaskRoiVec);


    void DrawLine(Mat &img, Point start, Point end, int colorScalar_B,int colorScalar_G,int colorScalar_R);

    bool
    generateObjPntsInCalibBoard(const int num_img_corner_detected, const double square_size, const cv::Size board_size,
                                std::vector<std::vector<cv::Point3f>> &corner_in_caliboard,
                                std::vector<std::vector<cv::Point2f>> &corner_in_caliboard2f);

	bool generateObjPntsInCalibBoardFishEye(const int num_img_corner_detected, const double square_size, const cv::Size board_size,
			std::vector<std::vector<cv::Point3d>> &corner_in_caliboard3d,
			std::vector<std::vector<cv::Point2d>> &corner_in_caliboard2d);

    bool computeReprojectionError(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                  const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                  const std::vector<std::vector<cv::Point2f>> &image_corner_pnts,
                                  const std::vector<std::vector<cv::Point3f>> &object_points);

    bool computeReprojectionErrorFishEye(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                                             const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                                             const std::vector<std::vector<cv::Point2d>> &image_corner_pnts,
                                                             const std::vector<std::vector<cv::Point3d>> &object_points);


    //single cam calibFiles write
    static bool WriteSingleCalibFilesPinHole(const std::string &calib_file_name, const PinholeCamPara &camCalibPara);

    //single cam calibFiles read
    static bool ReadSingleCalibFilesPinHole(const std::string &calib_file_name, PinholeCamPara &camCalibPara);


    //stereo calibFiles wrtie
    static bool WriteStereoCalibFilesPinHole(const std::string &calib_file_name, const PinholeCamPara &camL,const string serial_num_L,
                                                          const PinholeCamPara &camR,const string serial_num_R,
                                                          const RT &stereo_RT);

    static bool WriteStereoCalibFilesFishEye(const std::string &calib_file_name, const FishEyeCamPara &camL,const string serial_num_L,
                                             const FishEyeCamPara &camR,const string serial_num_R,
                                             const RT &stereo_RT);

    static bool WriteCaliboard2CamPose(const std::string &caliboard_to_cam_pose_name,const  std::vector<cv::Mat> tvecsMat,

    const std::vector<cv::Mat> rvecsMat);




    //stereo calibFiles read
    static bool
    ReadStereoCalibFilesPinHole(const std::string &calib_file_name, PinholeCamPara &camL, PinholeCamPara &camR,
                                RT &stereo_RT);

    void ecovacs_initUndistortRectifyMap(cv::Mat _cameraMatrix, cv::Mat _distCoeffs,
                                         cv::Mat _matR, cv::Mat _newCameraMatrix,
                                         Size size, int m1type, cv::Mat &_map1, cv::Mat &_map2);

    cv::Mat localUndistortImgPinHole(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef);

    cv::Mat localUndistortImgFishEye(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef);


    void
    verifyCalibrationPinHoleOK(std::vector<cv::Mat> imgSrcVec, PinholeCamPara &camPara, std::vector<cv::Mat> &imgUndistortVec);

    void
    verifyCalibrationFishEyeOK(std::vector<cv::Mat> imgSrcVec, FishEyeCamPara &camPara, std::vector<cv::Mat> &imgUndistortVec);


    void undistortImg(const cv::Mat &src, const cv::Mat &dst, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                      const cv::Mat &R);

public:

    calibCameraLR _camera; //distinguish L or R

    string _local_image_path; //image read path

    string _configPath; //ini

    string _resultPath; //calib result path to save

    string _serial_num; //serial num

    int _image_count; //calib image num

    std::vector<pair<int, vector<Point2f>>> _pair_corner_pnts_image_vec; //for pin hole

	std::vector<pair<int, vector<Point2d>>> _pair_corner_pnt2ds_image_vec; //for fisheye

    PinholeCamPara _pinholeCamPara; //for pin hole calib

    FishEyeCamPara _fishEyeCamPara; //for fish eye calib

    CalibrationData _calibrationData;

    RT _stereo_RT; //stereoCalib result

    cv::Mat _cameraMatrix_L,_DistortCoeff_L;
    cv::Mat _cameraMatrix_R,_DistortCoeff_R;

};


#endif // INTRINSIC_H_INCLUDED

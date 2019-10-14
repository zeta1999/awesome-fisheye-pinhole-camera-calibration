
#include "SharedHead.h"
#include "calibCoreAlgorithm.h"

using namespace cv;
using namespace std;


bool calibCoreAlgorithm::doSingleCalibratePinHole() {

    //step-1:read config files
    std::cout << "get in read config files" << endl;
    cv::FileStorage fs;
    fs.open(_configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open" << _configPath << std::endl;
        return false;
    }

    std::cout << "read imageType " << std::endl;

    //0:yuv /1:png/jpg
    int imageType = (int) fs["imageType"];

    double square_size=(double)fs["square_size"];


    cv::FileNode image_size = fs["image_size"];
    cv::FileNodeIterator it_initImg_size = image_size.begin();
    int image_size_width = (int) (*it_initImg_size)["width"];
    int image_size_height = (int) (*it_initImg_size)["height"];


    cv::FileNode board_size = fs["board_size"];
    cv::FileNodeIterator it_board_size = board_size.begin();
    int board_width = (int) (*it_board_size)["width"];
    int board_height = (int) (*it_board_size)["height"];
    _calibrationData._boardSize = cv::Size(board_width, board_height);

    std::cout << "read resultStorage" << std::endl;
    int resultStorage = (int) fs["resultStorage"];

    if(_camera._cameraL&&_camera._cameraR==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathL"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numL"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    if(_camera._cameraR&&_camera._cameraL==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathR"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numR"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    _resultPath=(string)fs["resultPath"];
    //cameraMatrix
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));

    //cameraDistortion
    cv::Mat distCoeffs = cv::Mat(1, 4, CV_64FC1, cv::Scalar::all(0));

    std::cout << "read config success.." << std::endl;
    fs.release();


    //step-2:read image
    ifstream f_img;
    f_img.open(_local_image_path.c_str());
    std::cout << "_local_image_path== " << _local_image_path.c_str() << endl;
    if (!f_img) {
        std::cerr << "Failed to open" << _local_image_path << endl;
        return false;
    }

    //read image to vector<Mat>
    std::vector<cv::Mat> imageVec;
    cv::Mat src_image;
    while (!f_img.eof()) {
        string s;
        getline(f_img, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            string img_path, img_prefix, img_name;
            ss >> img_prefix;
            ss >> img_name;
            img_path = img_prefix + "/" + img_name;

            FILE *f;
            unsigned char *pBuf = NULL;
            if (imageType == 0) {
                if (!(f = fopen(img_path.c_str(), "rb"))) {
                    std::cerr << "Failed to open " << img_path << std::endl;

                    return false;
                }
                pBuf = new unsigned char[image_size_width * image_size_height * 2];

                fseek(f, 0, SEEK_SET);
                int readfile = fread(pBuf, 1, image_size_width * image_size_height * 2, f);
                src_image = cv::Mat(image_size_height + image_size_height / 2, image_size_width, CV_8UC1, pBuf);
                cv::cvtColor(src_image, src_image, CV_YUV420sp2RGB);
            } else {
                src_image = cv::imread(img_path);
                if (src_image.empty()) {
                    std::cerr << "Failed to open" << img_path << std::endl;
                    return false;
                }

            }
            imageVec.push_back(src_image);
        }//if empty
    }//while
    f_img.close();

    std::cout << "imageVec.size()== " << imageVec.size() << std::endl;

    if (imageVec.size() == 0) {
        std::cerr << "no image to detect corners " << std::endl;
        return false;
    }

    pair<int, std::vector<cv::Point2f>> pair_corner_pnts_image;
    std::vector<std::vector<cv::Point2f>> image_corner_pnts_vec;
    image_corner_pnts_vec.clear();

    vector<cv::Point2f> image_corner_pnts;
    //step-3: detect corners
    for (int image_count = 0; image_count < imageVec.size(); ++image_count) {
        _image_count = (image_count + 1); //save detected image needed
        image_corner_pnts.clear();

        std::cout << "image_count== " << image_count + 1 << std::endl;

        if (!calibCoreAlgorithm::detectCornersPinhole(imageVec[image_count], _calibrationData._boardSize,
                                                      image_corner_pnts)) {
            continue;
        }
        image_corner_pnts_vec.push_back(image_corner_pnts);
        pair_corner_pnts_image.first = image_count;
        pair_corner_pnts_image.second = image_corner_pnts;
        _pair_corner_pnts_image_vec.push_back(pair_corner_pnts_image);
    }

    std::cout << "image detected success num= " << _pair_corner_pnts_image_vec.size() << std::endl;
    std::cout << "image_corner_pnts_vec== " << image_corner_pnts_vec.size() << std::endl;
    std::cout << "image_corner_pnts_vec[0].size()== " << image_corner_pnts_vec[0].size() << std::endl;

    _calibrationData._image_corner_pnt2fs_vec = image_corner_pnts_vec;

    //step-4:create corner in caliboard
    std::vector<std::vector<cv::Point3f>> corner_in_caliboard;
    if (!calibCoreAlgorithm::generateObjPntsInCalibBoard(_pair_corner_pnts_image_vec.size(), square_size,
                                                         _calibrationData._boardSize, corner_in_caliboard,
                                                         _calibrationData._plane2fPntsVec)) {
        std::cerr << "create corner in caliboard failed" << std::endl;
        return false;
    }
    std::cout << "corner_in_caliboard== " << corner_in_caliboard.size() << std::endl;
    _calibrationData._plane3fPntsVec = corner_in_caliboard;
    //step-4: single cam calib
    std::cout << "get to calibCamera() success" << endl;
    _calibrationData._imageSize = cv::Size(imageVec[0].cols, imageVec[0].rows);
    std::cout << "image_size== " << _calibrationData._imageSize << std::endl;

    if (!calibCoreAlgorithm::ZhangCalibrationMethodPinHole(_calibrationData, _pinholeCamPara)) {
        return false;
    }

    //yong.qi addded for test
    std::vector<cv::Mat> imgUndistortImgVec;
    verifyCalibrationPinHoleOK(imageVec, _pinholeCamPara, imgUndistortImgVec);

    return true;
}//end


bool calibCoreAlgorithm::doSingleCalibratePinHole_UseGuess() {

    //step-1:read config files
    std::cout << "get in read config files" << endl;
    cv::FileStorage fs;
    fs.open(_configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open" << _configPath << std::endl;
        return false;
    }

    std::cout << "read imageType " << std::endl;

    //0:yuv /1:png/jpg
    int imageType = (int) fs["imageType"];

    double square_size=(double)fs["square_size"];

    cv::FileNode image_size = fs["image_size"];
    cv::FileNodeIterator it_initImg_size = image_size.begin();
    int image_size_width = (int) (*it_initImg_size)["width"];
    int image_size_height = (int) (*it_initImg_size)["height"];


    cv::FileNode board_size = fs["board_size"];
    cv::FileNodeIterator it_board_size = board_size.begin();
    int board_width = (int) (*it_board_size)["width"];
    int board_height = (int) (*it_board_size)["height"];
    _calibrationData._boardSize = cv::Size(board_width, board_height);

    std::cout << "read resultStorage" << std::endl;
    int resultStorage = (int) fs["resultStorage"];

    if(_camera._cameraL&&_camera._cameraR==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathL"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numL"];
        std::cout<<"_serial_num"<<_serial_num<<endl;

        //read cameraMatrix
        fs["cam_matrix_L"]>>_pinholeCamPara._cameraMatrix;
        std::cout<<"cam_matrix_L"<<_pinholeCamPara._cameraMatrix<<endl;

        //read cameraDistortion
        fs["cam_distortion_L"]>>_pinholeCamPara._distortMatrix;
        std::cout<<"cam_distortion_L"<<_pinholeCamPara._distortMatrix<<endl;

    }
    if(_camera._cameraR&&_camera._cameraL==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathR"];
        //read serial num
        _serial_num=(string)fs["serial_numR"];
        //read cameraMatrix
        fs["cam_matrix_R"]>>_pinholeCamPara._cameraMatrix;
        //read cameraDistortion
        fs["cam_distortion_R"]>>_pinholeCamPara._distortMatrix;

    }
    //read stereo
    fs["system_rotation"]>>_stereo_RT._R;

    fs["system_translation"]>>_stereo_RT._T;

    _resultPath=(string)fs["resultPath"];

    std::cout << "read config success.." << std::endl;
    fs.release();


    //step-2:read image
    ifstream f_img;
    f_img.open(_local_image_path.c_str());
    std::cout << "_local_image_path== " << _local_image_path.c_str() << endl;
    if (!f_img) {
        std::cerr << "Failed to open" << _local_image_path << endl;
        return false;
    }

    //read image to vector<Mat>
    std::vector<cv::Mat> imageVec;
    cv::Mat src_image;
    while (!f_img.eof()) {
        string s;
        getline(f_img, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            string img_path, img_prefix, img_name;
            ss >> img_prefix;
            ss >> img_name;
            img_path = img_prefix + "/" + img_name;

            FILE *f;
            unsigned char *pBuf = NULL;
            if (imageType == 0) {
                if (!(f = fopen(img_path.c_str(), "rb"))) {
                    std::cerr << "Failed to open " << img_path << std::endl;

                    return false;
                }
                pBuf = new unsigned char[image_size_width * image_size_height * 2];

                fseek(f, 0, SEEK_SET);
                int readfile = fread(pBuf, 1, image_size_width * image_size_height * 2, f);
                src_image = cv::Mat(image_size_height + image_size_height / 2, image_size_width, CV_8UC1, pBuf);
                cv::cvtColor(src_image, src_image, CV_YUV420sp2RGB);
            } else {
                src_image = cv::imread(img_path);
                if (src_image.empty()) {
                    std::cerr << "Failed to open" << img_path << std::endl;
                    return false;
                }

            }
            imageVec.push_back(src_image);
        }//if empty
    }//while
    f_img.close();

    if (imageVec.size() == 0) {
        std::cerr << "no image to detect corners " << std::endl;
        return false;
    }

    pair<int, std::vector<cv::Point2f>> pair_corner_pnts_image;
    std::vector<std::vector<cv::Point2f>> image_corner_pnts_vec;
    image_corner_pnts_vec.clear();

    vector<cv::Point2f> image_corner_pnts;
    //step-3: detect corners
    for (int image_count = 0; image_count < imageVec.size(); ++image_count) {
        _image_count = (image_count + 1); //save detected image needed
        image_corner_pnts.clear();

        std::cout << "image_count== " << image_count + 1 << std::endl;

        if (!calibCoreAlgorithm::detectCornersPinhole(imageVec[image_count], _calibrationData._boardSize,
                                                      image_corner_pnts)) {
            continue;
        }
        image_corner_pnts_vec.push_back(image_corner_pnts);
        pair_corner_pnts_image.first = image_count;
        pair_corner_pnts_image.second = image_corner_pnts;
        _pair_corner_pnts_image_vec.push_back(pair_corner_pnts_image);
    }

    std::cout << "image detected success num= " << _pair_corner_pnts_image_vec.size() << std::endl;
    std::cout << "image_corner_pnts_vec== " << image_corner_pnts_vec.size() << std::endl;
    std::cout << "image_corner_pnts_vec[0].size()== " << image_corner_pnts_vec[0].size() << std::endl;

    _calibrationData._image_corner_pnt2fs_vec = image_corner_pnts_vec;

    //step-4:create corner in caliboard
    std::vector<std::vector<cv::Point3f>> corner_in_caliboard;
    if (!calibCoreAlgorithm::generateObjPntsInCalibBoard(_pair_corner_pnts_image_vec.size(), square_size,
                                                         _calibrationData._boardSize, corner_in_caliboard,
                                                         _calibrationData._plane2fPntsVec)) {
        std::cerr << "create corner in caliboard failed" << std::endl;
        return false;
    }
    std::cout << "corner_in_caliboard== " << corner_in_caliboard.size() << std::endl;
    _calibrationData._plane3fPntsVec = corner_in_caliboard;
    //step-4: single cam calib
    std::cout << "get to calibCamera() success" << endl;
    _calibrationData._imageSize = cv::Size(imageVec[0].cols, imageVec[0].rows);
    std::cout << "image_size== " << _calibrationData._imageSize << std::endl;

    if (!calibCoreAlgorithm::ZhangCalibrationMethodPinHole_UseGuess(_calibrationData, _pinholeCamPara)) {
        return false;
    }

    //yong.qi addded for test
    std::vector<cv::Mat> imgUndistortImgVec;
    verifyCalibrationPinHoleOK(imageVec, _pinholeCamPara, imgUndistortImgVec);

    return true;
}//end





bool calibCoreAlgorithm::doSingleCalibrateFishEye() {
    //step-1:read config files
    std::cout << "get in read config files" << endl;
    cv::FileStorage fs;
    fs.open(_configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open" << _configPath << std::endl;
        return false;
    }

    std::cout << "read imageType " << std::endl;

    //0:yuv /1:png/jpg
    int imageType = (int) fs["imageType"];

    double square_size=(double)fs["square_size"];


    cv::FileNode image_size = fs["image_size"];
    cv::FileNodeIterator it_initImg_size = image_size.begin();
    int image_size_width = (int) (*it_initImg_size)["width"];
    int image_size_height = (int) (*it_initImg_size)["height"];


    cv::FileNode board_size = fs["board_size"];
    cv::FileNodeIterator it_board_size = board_size.begin();
    int board_width = (int) (*it_board_size)["width"];
    int board_height = (int) (*it_board_size)["height"];
    _calibrationData._boardSize = cv::Size(board_width, board_height);

    std::cout << "read resultStorage" << std::endl;
    int resultStorage = (int) fs["resultStorage"];

    if(_camera._cameraL&&_camera._cameraR==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathL"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numL"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    if(_camera._cameraR&&_camera._cameraL==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathR"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numR"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    _resultPath=(string)fs["resultPath"];
    //cameraMatrix
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));

    //cameraDistortion
    cv::Mat distCoeffs = cv::Mat(4, 1, CV_64FC1, cv::Scalar::all(0));

    std::cout << "read config success.." << std::endl;
    fs.release();


    //step-2:read image
    ifstream f_img;
    f_img.open(_local_image_path.c_str());
    std::cout << "_local_image_path== " << _local_image_path.c_str() << endl;
    if (!f_img) {
        std::cerr << "Failed to open" << _local_image_path << endl;
        return false;
    }

    //read image to vector<Mat>
    std::vector<cv::Mat> imageVec;
    cv::Mat src_image;
    while (!f_img.eof()) {
        string s;
        getline(f_img, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            string img_path, img_prefix, img_name;
            ss >> img_prefix;
            ss >> img_name;
            img_path = img_prefix + "/" + img_name;

            FILE *f;
            unsigned char *pBuf = NULL;
            if (imageType == 0) {
                if (!(f = fopen(img_path.c_str(), "rb"))) {
                    std::cerr << "Failed to open " << img_path << std::endl;

                    return false;
                }
                pBuf = new unsigned char[image_size_width * image_size_height * 2];

                fseek(f, 0, SEEK_SET);
                int readfile = fread(pBuf, 1, image_size_width * image_size_height * 2, f);
                src_image = cv::Mat(image_size_height + image_size_height / 2, image_size_width, CV_8UC1, pBuf);
                cv::cvtColor(src_image, src_image, CV_YUV420sp2RGB);
            } else {
                src_image = cv::imread(img_path);
                if (src_image.empty()) {
                    std::cerr << "Failed to open" << img_path << std::endl;
                    return false;
                }

            }
            imageVec.push_back(src_image);
        }//if empty
    }//while
    f_img.close();

    std::cout << "imageVec.size()== " << imageVec.size() << std::endl;

    if (imageVec.size() == 0) {
        std::cerr << "no image to detect corners " << std::endl;
        return false;
    }

    pair<int, std::vector<cv::Point2d>> pair_corner_pnt2ds_image;
    std::vector<std::vector<cv::Point2d>> image_corner_pnt2ds_vec;
    image_corner_pnt2ds_vec.clear();

    //vector<cv::Point2f> image_corner_pnts;
    vector<cv::Point2d> image_corner_pnt2ds;
    vector<cv::Point2f> image_corner_pnt2fs;
    //step-3: detect corners
    for (int image_count = 0; image_count < imageVec.size(); ++image_count) {
        _image_count = (image_count + 1); //save detected image needed
        image_corner_pnt2ds.clear();
        image_corner_pnt2fs.clear();
        //for fish eye
        if (!calibCoreAlgorithm::detectCornersFishEye(imageVec[image_count], _calibrationData._boardSize,
                                                      image_corner_pnt2ds)) {
            continue;
        }
        image_corner_pnt2ds_vec.push_back(image_corner_pnt2ds);
        pair_corner_pnt2ds_image.first = image_count;
        pair_corner_pnt2ds_image.second = image_corner_pnt2ds;
        _pair_corner_pnt2ds_image_vec.push_back(pair_corner_pnt2ds_image);
    }

    std::cout << "image detected success num= " << _pair_corner_pnt2ds_image_vec.size() << std::endl;

//    //for test write corners detected to local
//    for (int i = 0; i < _pair_corner_pnt2ds_image_vec.size(); ++i)
//    {
//        char* fname="../corners_detected.txt";
//        ofstream fout_corner(fname,ios::app);
//        fout_corner<<_pair_corner_pnt2ds_image_vec[i].first<<endl;
//        for (int j = 0; j < _pair_corner_pnt2ds_image_vec[i].second.size(); ++j)
//        {
//            fout_corner<<_pair_corner_pnt2ds_image_vec[i].second[j]<<endl;
//        }
//        fout_corner<<endl<<endl<<endl;
//    }
//    //ended

    _calibrationData._image_corner_pnt2ds_vec = image_corner_pnt2ds_vec;

    //step-4:create corner in caliboard
    std::vector<std::vector<cv::Point3d>> corner_in_caliboard;
    if (!calibCoreAlgorithm::generateObjPntsInCalibBoardFishEye(_pair_corner_pnt2ds_image_vec.size(), square_size,
                                                                _calibrationData._boardSize, corner_in_caliboard,
                                                                _calibrationData._plane2dPntsVec)) {
        std::cerr << "create corner in caliboard failed" << std::endl;
        return false;
    }
    std::cout << "corner_in_caliboard== " << corner_in_caliboard.size() << std::endl;
    _calibrationData._plane3dPntsVec = corner_in_caliboard;
    //step-4: single cam calib
    std::cout << "get to calibCamera() success" << endl;
    _calibrationData._imageSize = cv::Size(imageVec[0].cols, imageVec[0].rows);
    std::cout << "image_size== " << _calibrationData._imageSize << std::endl;

    if (!calibCoreAlgorithm::ZhangCalibrationKannalaBrandt(_calibrationData, _fishEyeCamPara)) {
        return false;
    }

    //yong.qi addded for test
    std::vector<cv::Mat> imgUndistortImgVec;
    verifyCalibrationFishEyeOK(imageVec, _fishEyeCamPara, imgUndistortImgVec);

    return true;

}

bool calibCoreAlgorithm::doSingleCalibrateFishEyeROI() {

    std::cout<<"begin to read config files... "<<endl;
    //step-1:read config files
    cv::FileStorage fs;
    fs.open(_configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open" << _configPath << std::endl;
        return false;
    }

    std::cout<<"open file is OK"<<std::endl;

    if(_camera._cameraL&&_camera._cameraR==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathL"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numL"];
        std::cout<<"_serial_numL"<<_serial_num<<endl;

        //read cameraMatrix
        fs["cam_matrix_L"]>>_fishEyeCamPara._cameraMatrix;
        std::cout<<"cam_matrix_L"<<_fishEyeCamPara._cameraMatrix<<endl;

        //read cameraDistortion
        fs["cam_distortion_L"]>>_fishEyeCamPara._DistortCoeff;
        std::cout<<"cam_distortion_L"<<_fishEyeCamPara._DistortCoeff<<endl;

        //for test
        _fishEyeCamPara._cameraMatrix.copyTo(_cameraMatrix_L);
        _fishEyeCamPara._DistortCoeff.copyTo(_DistortCoeff_L);
        //ended
    }
    if(_camera._cameraR&&_camera._cameraL==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathR"];
        std::cout<<" _local_image_pathR"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numR"];
        std::cout<<"_serial_num"<<_serial_num<<endl;

        //read cameraMatrix
        fs["cam_matrix_R"]>>_fishEyeCamPara._cameraMatrix;
        std::cout<<"cam_matrix_R"<<_fishEyeCamPara._cameraMatrix<<endl;

        //read cameraDistortion
        fs["cam_distortion_R"]>>_fishEyeCamPara._DistortCoeff;
        std::cout<<"cam_distortion_R"<<_fishEyeCamPara._DistortCoeff<<endl;

        //for test
        _fishEyeCamPara._cameraMatrix.copyTo(_cameraMatrix_R);
        _fishEyeCamPara._DistortCoeff.copyTo(_DistortCoeff_R);
        //ended
    }

    //read stereo
    fs["system_rotation"]>>_stereo_RT._R;
    std::cout<<"system_rotation"<<_stereo_RT._R<<endl;

    fs["system_translation"]>>_stereo_RT._T;
    std::cout<<"system_translation"<<_stereo_RT._T<<endl;

    _resultPath=(string)fs["resultPath"];

    int imageType=(int)fs["imageType"];

    cv::FileNode image_size=fs["image_size"];
    cv::FileNodeIterator it_image_size=image_size.begin();
    int image_size_width=(int)(*it_image_size)["width"];
    int image_size_height=(int)(*it_image_size)["height"];

    cv::FileNode board_size = fs["board_size"];
    cv::FileNodeIterator it_board_size = board_size.begin();
    int board_width = (int) (*it_board_size)["width"];
    int board_height = (int) (*it_board_size)["height"];
    _calibrationData._boardSize = cv::Size(board_width, board_height);

    double square_size = (double) fs["square_size"];

    std::cout << "read resultStorage" << std::endl;
    int resultStorage = (int) fs["resultStorage"];

    std::cout << "read config success.." << std::endl;
    fs.release();

    //step-2:read image
    ifstream f_img;
    f_img.open(_local_image_path.c_str());
    std::cout << "_local_image_path== " << _local_image_path.c_str() << endl;
    if (!f_img) {
        std::cerr << "Failed to open" << _local_image_path << endl;
        return false;
    }

    //read image to vector<Mat>
    std::vector<cv::Mat> imageVec;
    cv::Mat src_image;
    while (!f_img.eof()) {
        string s;
        getline(f_img, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            string img_path, img_prefix, img_name;
            ss >> img_prefix;
            ss >> img_name;
            img_path = img_prefix + "/" + img_name;

            FILE *f;
            unsigned char *pBuf = NULL;
            if (imageType == 0) {
                if (!(f = fopen(img_path.c_str(), "rb"))) {
                    std::cerr << "Failed to open " << img_path << std::endl;
                    return false;
                }
                pBuf = new unsigned char[image_size_width * image_size_height * 2];

                fseek(f, 0, SEEK_SET);
                int readfile = fread(pBuf, 1, image_size_width * image_size_height * 2, f);
                src_image = cv::Mat(image_size_height + image_size_height / 2, image_size_width, CV_8UC1, pBuf);
                cv::cvtColor(src_image, src_image, CV_YUV420sp2RGB);
            } else {
                src_image = cv::imread(img_path);
                if (src_image.empty()) {
                    std::cerr << "Failed to open" << img_path << std::endl;
                    return false;
                }
            }
            imageVec.push_back(src_image);
        }//if empty
    }//while
    f_img.close();

    std::cout << "imageVec.size()== " << imageVec.size() << std::endl;

    if (imageVec.size() == 0) {
        std::cerr << "no image to detect corners " << std::endl;
        return false;
    }

    pair<int, std::vector<cv::Point2d>> pair_corner_pnt2ds_image;
    std::vector<std::vector<cv::Point2d>> image_corner_pnt2ds_vec;
    image_corner_pnt2ds_vec.clear();

    //vector<cv::Point2f> image_corner_pnts;
    vector<cv::Point2d> image_corner_pnt2ds;
    vector<cv::Point2f> image_corner_pnt2fs;
    //step-3: detect corners
    for (int image_count = 0; image_count < imageVec.size(); ++image_count) {
         //save detected image needed
        cv::Mat imageGray;
        if (imageVec[image_count].channels() == 1) {
            imageVec[image_count].copyTo(imageGray);
        } else {
            cvtColor(imageVec[image_count], imageGray, CV_BGR2GRAY);
        }
        //add blur
        cv::GaussianBlur(imageGray, imageGray, cv::Size(5, 5), 0);

        std::vector<cv::Rect> maskRoiVec;

        //calibCoreAlgorithm::createROI(imageGray, 9, maskRoiVec);
        calibCoreAlgorithm::createROI_test(imageGray, 2, maskRoiVec);

        for (int roi_count = 0; roi_count < maskRoiVec.size(); roi_count++) {
            _image_count++;
            cv::Rect maskRoi = maskRoiVec[roi_count];
            if (!calibCoreAlgorithm::detectCornersFishEyeROI(imageGray, maskRoi, _calibrationData._boardSize,
                                                             image_corner_pnt2ds))
            {
                maskRoi.x-=20;
                maskRoi.y-=20;
                maskRoi.width+=40;
                maskRoi.height+=40;
                if(!calibCoreAlgorithm::detectCornersFishEyeROI(imageGray, maskRoi, _calibrationData._boardSize,
                                                                image_corner_pnt2ds))
                {
                    std::cerr<<"after reset roi, still can not detect corners successfully"<<endl;
                    continue;
                }

            }
            image_corner_pnt2ds_vec.push_back(image_corner_pnt2ds);
            pair_corner_pnt2ds_image.first =_image_count;
            pair_corner_pnt2ds_image.second = image_corner_pnt2ds;
            _pair_corner_pnt2ds_image_vec.push_back(pair_corner_pnt2ds_image);

        }//roi_count

    }//image_count

    std::cout << "image detected success num= " << _pair_corner_pnt2ds_image_vec.size() << std::endl;

//    //for test write corners detected to local
//    for (int i = 0; i < _pair_corner_pnt2ds_image_vec.size(); ++i)
//    {
//        char* fname="../corners_detected.txt";
//        ofstream fout_corner(fname,ios::app);
//        fout_corner<<_pair_corner_pnt2ds_image_vec[i].first<<endl;
//        for (int j = 0; j < _pair_corner_pnt2ds_image_vec[i].second.size(); ++j)
//        {
//            fout_corner<<_pair_corner_pnt2ds_image_vec[i].second[j]<<endl;
//        }
//        fout_corner<<endl<<endl<<endl;
//    }
//    //ended

    _calibrationData._image_corner_pnt2ds_vec = image_corner_pnt2ds_vec;

    //step-4:create corner in caliboard
    std::vector<std::vector<cv::Point3d>> corner_in_caliboard;
    if (!calibCoreAlgorithm::generateObjPntsInCalibBoardFishEye(_pair_corner_pnt2ds_image_vec.size(), square_size,
                                                                _calibrationData._boardSize, corner_in_caliboard,
                                                                _calibrationData._plane2dPntsVec)) {
        std::cerr << "create corner in caliboard failed" << std::endl;
        return false;
    }
    std::cout << "corner_in_caliboard== " << corner_in_caliboard.size() << std::endl;
    _calibrationData._plane3dPntsVec = corner_in_caliboard;
    //step-4: single cam calib
    std::cout << "get to calibCamera() success" << endl;
    _calibrationData._imageSize = cv::Size(imageVec[0].cols, imageVec[0].rows);
    std::cout << "image_size== " << _calibrationData._imageSize << std::endl;

    if (!calibCoreAlgorithm::ZhangCalibrationKannalaBrandtROI(_calibrationData)) {
        return false;
    }

    //yong.qi addded for test
    std::vector<cv::Mat> imgUndistortImgVec;
    verifyCalibrationFishEyeOK(imageVec, _fishEyeCamPara, imgUndistortImgVec);

    return true;
}


bool calibCoreAlgorithm::ZhangCalibrationMethodPinHole(CalibrationData &cData, PinholeCamPara &campara) {
    //cv::Mat cameraMatrix, distCoeffMat;

    double pinholeCalib_rms = calibrateCamera(cData._plane3fPntsVec, cData._image_corner_pnt2fs_vec, cData._imageSize,
                                              campara._cameraMatrix, campara._distortMatrix, cData._rvecsMat,
                                              cData._tvecsMat, 0,
                                              TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));
    campara._imgRTVec.clear();

    std::cout << "ZhangCalibrationMethodPinHole for cameraMatrix == " << campara._cameraMatrix << std::endl;
    std::cout << "ZhangCalibrationMethodPinHole for distortCoeff== " << campara._distortMatrix << std::endl;

    //yong.qi for test show
    cv::Mat cameraMatrix = campara._cameraMatrix;
    cv::Mat distortMatrix = campara._distortMatrix;
    //ended

    for (size_t h = 0; h < _calibrationData._rvecsMat.size(); h++) {
        RT rt;
        rt._R = _calibrationData._rvecsMat[h];
        rt._T = _calibrationData._tvecsMat[h];
        campara._imgRTVec.push_back(rt);
    }

    std::cout << "cameraMatrix== " << campara._cameraMatrix << endl;
    std::cout << "distCoeffs== " << campara._distortMatrix << endl;

    std::cout << "pinholeCalib_rms== " << pinholeCalib_rms << std::endl;

    //step-5: computeReprojectError
    if (!calibCoreAlgorithm::computeReprojectionError(campara._cameraMatrix, campara._distortMatrix, cData._rvecsMat,
                                                      cData._tvecsMat, cData._image_corner_pnt2fs_vec,
                                                      cData._plane3fPntsVec)) {
        std::cerr << "failed to computeReprojectionError.. " << std::endl;
        return false;
    }
    std::cout << "reprojectionError[0]== " << campara._ReprojectionError[0];
    std::cout << "reprojectionError[1]== " << campara._ReprojectionError[1];

    double totalNum = sqrt(pow(campara._ReprojectionError[0], 2) + pow(campara._ReprojectionError[1], 2));
    std::cout << "totalNum== " << totalNum << std::endl;

    std::cout << "totalReproNormErr== " << campara._totalReproNormErr << endl;

    return true;
}

bool calibCoreAlgorithm::ZhangCalibrationMethodPinHole_UseGuess(CalibrationData &cData, PinholeCamPara &campara)
{

    //for test
    std::cout<<"before calib campara._cameraMatrix== "<<campara._cameraMatrix<<endl;
    std::cout<<"before calib campara._distortMatrix== "<<campara._distortMatrix<<endl;
    //ended

    double pinholeCalib_rms = calibrateCamera(cData._plane3fPntsVec, cData._image_corner_pnt2fs_vec, cData._imageSize,
                                              campara._cameraMatrix, campara._distortMatrix, cData._rvecsMat,
                                              cData._tvecsMat, cv::CALIB_USE_INTRINSIC_GUESS+cv::CALIB_FIX_K3,
                                              TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));
    campara._imgRTVec.clear();

    std::cout << "after ZhangCalib  cameraMatrix == " << campara._cameraMatrix << std::endl;
    std::cout << "after ZhangCalib  distortCoeff== " << campara._distortMatrix << std::endl;

    for (size_t h = 0; h < _calibrationData._rvecsMat.size(); h++) {
        RT rt;
        rt._R = _calibrationData._rvecsMat[h];
        rt._T = _calibrationData._tvecsMat[h];
        campara._imgRTVec.push_back(rt);
    }

    std::cout << "cameraMatrix== " << campara._cameraMatrix << endl;
    std::cout << "distCoeffs== " << campara._distortMatrix << endl;

    std::cout << "pinholeCalib_rms== " << pinholeCalib_rms << std::endl;

    //step-5: computeReprojectError
    if (!calibCoreAlgorithm::computeReprojectionError(campara._cameraMatrix, campara._distortMatrix, cData._rvecsMat,
                                                      cData._tvecsMat, cData._image_corner_pnt2fs_vec,
                                                      cData._plane3fPntsVec)) {
        std::cerr << "failed to computeReprojectionError.. " << std::endl;
        return false;
    }
    std::cout << "reprojectionError[0]== " << campara._ReprojectionError[0];
    std::cout << "reprojectionError[1]== " << campara._ReprojectionError[1];

    double totalNum = sqrt(pow(campara._ReprojectionError[0], 2) + pow(campara._ReprojectionError[1], 2));
    std::cout << "totalNum== " << totalNum << std::endl;

    std::cout << "totalReproNormErr== " << campara._totalReproNormErr << endl;

    return true;

}





bool calibCoreAlgorithm::ZhangCalibrationKannalaBrandt(CalibrationData &cData, FishEyeCamPara &camPara) {

    std::cout<<"get in ZhangCalibrationKannalaBrandt()"<<endl;

    double fishEyeCalib_rms = fisheye::calibrate(cData._plane3dPntsVec, cData._image_corner_pnt2ds_vec,
                                                 cData._imageSize, camPara._cameraMatrix, camPara._DistortCoeff,
                                                 cData._rvecsMat, cData._tvecsMat,
                                                 cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_FIX_SKEW,
                                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

    //save caliboard coordinate to cam coordinate translate
   // std::string Caliboard2CamPosePath = "./caliboard2CamPose.yaml";
   // if (!calibCoreAlgorithm::WriteCaliboard2CamPose(Caliboard2CamPosePath, cData._tvecsMat, cData._rvecsMat)) {
     //   return false;
    //}

    std::cout << "ZhangCalibrationKannalaBrandt for cameraMatrix == " << camPara._cameraMatrix << std::endl;
    std::cout << "ZhangCalibrationKannalaBrandt for distortCoeff== " << camPara._DistortCoeff << std::endl;


    for (size_t h = 0; h < _calibrationData._rvecsMat.size(); h++) {
        RT rt;
        rt._R = _calibrationData._rvecsMat[h];
        rt._T = _calibrationData._tvecsMat[h];
        camPara._imgRTVec.push_back(rt);
    }

    std::cout << "cameraMatrix== " << camPara._cameraMatrix << endl;
    std::cout << "distCoeffs== " << camPara._DistortCoeff << endl;

    std::cout << "fishEyeCalib_rms== " << fishEyeCalib_rms << std::endl;

    //step-5: computeReprojectError
    if (!calibCoreAlgorithm::computeReprojectionErrorFishEye(camPara._cameraMatrix, camPara._DistortCoeff,
                                                             cData._rvecsMat, cData._tvecsMat,
                                                             cData._image_corner_pnt2ds_vec, cData._plane3dPntsVec)) {
        std::cerr << "failed to computeReprojectionError.. " << std::endl;
        return false;
    }
    std::cout << "reprojectionError[0]== " << camPara._ReprojectionError[0];
    std::cout << "reprojectionError[1]== " << camPara._ReprojectionError[1];

    double totalNum = sqrt(pow(camPara._ReprojectionError[0], 2) + pow(camPara._ReprojectionError[1], 2));

    std::cout << "totalNum== " << totalNum << std::endl;

    std::cout << "totalReproNormErr== " << camPara._totalReproNormErr << endl;

    return true;
}


bool calibCoreAlgorithm::ZhangCalibrationKannalaBrandtROI(CalibrationData &cData)
{

    std::cout<<"cData._plane3dPntsVec.size()== "<<cData._plane3dPntsVec.size()<<endl;


    std::cout<<"cData._image_corner_pnt2ds_vec.size()== "<<cData._image_corner_pnt2ds_vec.size()<<endl;


    std::cout<<"before calibrating _fishEyeCamPara._cameraMatrix== "<<_fishEyeCamPara._cameraMatrix<<endl;

    std::cout<<"before calibrating _fishEyeCamPara._DistortCoeff== "<<_fishEyeCamPara._DistortCoeff<<endl;


//    double fishEyeCalib_rms = fisheye::calibrate(cData._plane3dPntsVec, cData._image_corner_pnt2ds_vec,
//                                                 cData._imageSize, _fishEyeCamPara._cameraMatrix, _fishEyeCamPara._DistortCoeff,
//                                                 cData._rvecsMat, cData._tvecsMat,
//                                                 //cv::fisheye::CALIB_USE_INTRINSIC_GUESS,
//                                                /*cv::fisheye::CALIB_USE_INTRINSIC_GUESS+*/cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_FIX_SKEW,
//                                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

//    cv::Mat outFishEyeCamMatrix = cv::Mat::eye(3, 3, CV_32FC1);
//
//    cv::Mat outFishEyeCamDistort = cv::Mat::zeros(4, 1, CV_32FC1);
//
//   _fishEyeCamPara._cameraMatrix.copyTo(outFishEyeCamMatrix);
//
//   //outFishEyeCamDistort=(cv::Mat_<float>(4,1)<<-0.02724966413904360,0.01387972934505288,-0.01443299094096242,0.00220385605939798);
//
//    _fishEyeCamPara._DistortCoeff.copyTo(outFishEyeCamDistort);
//
//
//    std::cout<<"before calibrating outFishEyeCamMatrix== "<<outFishEyeCamMatrix<<endl;
//
//    std::cout<<"before calibrating outFishEyeCamDistort== "<<outFishEyeCamDistort<<endl;

    double fishEyeCalib_rms = fisheye::calibrate(cData._plane3dPntsVec, cData._image_corner_pnt2ds_vec,
                                                 cData._imageSize,_fishEyeCamPara._cameraMatrix ,_fishEyeCamPara._DistortCoeff,
                                                 cData._rvecsMat, cData._tvecsMat,
            cv::fisheye::CALIB_USE_INTRINSIC_GUESS+cv::fisheye::CALIB_FIX_SKEW+cv::fisheye::CALIB_CHECK_COND,
            //cv::fisheye::CALIB_USE_INTRINSIC_GUESS+cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_FIX_SKEW,
                                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-6));

    std::cout << "after KannalaBrandt  cameraMatrix == " << _fishEyeCamPara._cameraMatrix << std::endl;
    std::cout << "after KannalaBrandt  distortCoeff== " << _fishEyeCamPara._DistortCoeff << std::endl;

    std::cout << "fishEyeCalib_rms== " << fishEyeCalib_rms << std::endl;

    //step-5: computeReprojectError
    if (!calibCoreAlgorithm::computeReprojectionErrorFishEye(_fishEyeCamPara._cameraMatrix, _fishEyeCamPara._DistortCoeff,
                                                             cData._rvecsMat, cData._tvecsMat,
                                                             cData._image_corner_pnt2ds_vec, cData._plane3dPntsVec)) {
        std::cerr << "failed to computeReprojectionError.. " << std::endl;
        return false;
    }
    std::cout << "reprojectionError[0]== " << _fishEyeCamPara._ReprojectionError[0];
    std::cout << "reprojectionError[1]== " << _fishEyeCamPara._ReprojectionError[1];

    double totalNum = sqrt(pow(_fishEyeCamPara._ReprojectionError[0], 2) + pow(_fishEyeCamPara._ReprojectionError[1], 2));

    std::cout << "totalNum== " << totalNum << std::endl;

    std::cout << "totalReproNormErr== " << _fishEyeCamPara._totalReproNormErr << endl;

    return true;
}


bool calibCoreAlgorithm::StereoCalibPinHole(calibCoreAlgorithm &camL, calibCoreAlgorithm &camR, RT &stereo_RT) {
    //stereoCalibrate
    std::vector<std::vector<cv::Point3f>> calib_board_corners;
    std::vector<std::vector<Point2f>> image_L_pnts;
    std::vector<std::vector<Point2f>> image_R_pnts;
    cv::Mat R, T, E, F;
    for (size_t num_L = 0; num_L < camL._pair_corner_pnts_image_vec.size(); num_L++) {
        for (size_t num_R = 0; num_R < camR._pair_corner_pnts_image_vec.size(); num_R++) {
            if (camL._pair_corner_pnts_image_vec[num_L].first == camR._pair_corner_pnts_image_vec[num_R].first) {
                image_L_pnts.push_back(camL._pair_corner_pnts_image_vec[num_L].second);
                image_R_pnts.push_back(camR._pair_corner_pnts_image_vec[num_R].second);
                calib_board_corners.push_back(camL._calibrationData._plane3fPntsVec[num_L]);
            } else {
                continue;
            }
        }
    }
    std::cout << "image_L_pnts.size== " << image_L_pnts.size() << endl
              << "image_R_pnts== " << image_R_pnts.size() << endl;

    std::cout << "calib_board_corners.size== " << calib_board_corners.size() << endl;

    double stereo_rms = cv::stereoCalibrate(calib_board_corners, image_L_pnts, image_R_pnts,
                                            camL._pinholeCamPara._cameraMatrix, camL._pinholeCamPara._distortMatrix,
                                            camR._pinholeCamPara._cameraMatrix, camR._pinholeCamPara._distortMatrix,
                                            camL._calibrationData._imageSize, R, T, E, F,
                                            CV_CALIB_FIX_INTRINSIC + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 +
                                            CV_CALIB_FIX_K5,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                                                             1e-5));

    std::cout << "stereo rms == " << stereo_rms << std::endl;


    stereo_RT._R = R;
    stereo_RT._T = T;
    stereo_RT._stero_rms = stereo_rms;
    std::cout << "stereo_R== " << stereo_RT._R << endl;
    std::cout << "stereo_T== " << stereo_RT._T << endl;

    //yong.qi added
    double disBase = norm(stereo_RT._T, CV_L2);
    std::cout << "disBase== " << disBase << endl << endl;
    stereo_RT._length_T = disBase;
    //yong.qi ended

    //save calib files to local
    //save left cam calibFiles
//    std::string left_cam_para_path = "./leftCamPara.yaml";
//    if (!calibCoreAlgorithm::WriteSingleCalibFilesPinHole(left_cam_para_path, camL._pinholeCamPara)) {
//        return false;
//    }
//
//    //save right cam calibFiles
//    std::string right_cam_para_path = "./rightCamPara.yaml";
//    if (!calibCoreAlgorithm::WriteSingleCalibFilesPinHole(right_cam_para_path, camR._pinholeCamPara)) {
//        return false;
//    }

    //save stero cam calibFiles

    if (!calibCoreAlgorithm::WriteStereoCalibFilesPinHole(camL._resultPath, camL._pinholeCamPara,camL._serial_num,
                                                          camR._pinholeCamPara,camR._serial_num, stereo_RT)) {
        return false;
    }
    std::cout << "calib process success... " << std::endl;


    return true;

}

bool calibCoreAlgorithm::StereoCalibPinHole_UseGuess(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR)
{
    //stereoCalibrate
    std::vector<std::vector<cv::Point3f>> calib_board_corners;
    std::vector<std::vector<Point2f>> image_L_pnts;
    std::vector<std::vector<Point2f>> image_R_pnts;
    cv::Mat R, T, E, F;
    for (size_t num_L = 0; num_L < camL._pair_corner_pnts_image_vec.size(); num_L++) {
        for (size_t num_R = 0; num_R < camR._pair_corner_pnts_image_vec.size(); num_R++) {
            if (camL._pair_corner_pnts_image_vec[num_L].first == camR._pair_corner_pnts_image_vec[num_R].first) {
                image_L_pnts.push_back(camL._pair_corner_pnts_image_vec[num_L].second);
                image_R_pnts.push_back(camR._pair_corner_pnts_image_vec[num_R].second);
                calib_board_corners.push_back(camL._calibrationData._plane3fPntsVec[num_L]);
            } else {
                continue;
            }
        }
    }
    std::cout << "image_L_pnts.size== " << image_L_pnts.size() << endl
              << "image_R_pnts== " << image_R_pnts.size() << endl;

    std::cout << "calib_board_corners.size== " << calib_board_corners.size() << endl;

    double stereo_rms = cv::stereoCalibrate(calib_board_corners, image_L_pnts, image_R_pnts,
                                            camL._pinholeCamPara._cameraMatrix, camL._pinholeCamPara._distortMatrix,
                                            camR._pinholeCamPara._cameraMatrix, camR._pinholeCamPara._distortMatrix,
                                            camL._calibrationData._imageSize, camL._stereo_RT._R, camL._stereo_RT._T, E, F,
                                            cv::CALIB_USE_INTRINSIC_GUESS+ CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 +
                                            CV_CALIB_FIX_K5,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                                                             1e-5));

    std::cout << "stereo rms == " << stereo_rms << std::endl;

    camL._stereo_RT._stero_rms = stereo_rms;
    std::cout << "stereo_R== " << camL._stereo_RT._R << endl;
    std::cout << "stereo_T== " << camL._stereo_RT._T << endl;

    //yong.qi added
    double disBase = norm(camL._stereo_RT._T, CV_L2);
    std::cout << "disBase== " << disBase << endl << endl;
    camL._stereo_RT._length_T = disBase;
    //yong.qi ended

    //save stero cam calibFiles

    if (!calibCoreAlgorithm::WriteStereoCalibFilesPinHole(camL._resultPath, camL._pinholeCamPara,camL._serial_num,
                                                          camR._pinholeCamPara,camR._serial_num, camL._stereo_RT)) {
        return false;
    }
    std::cout << "calib process success... " << std::endl;


    return true;
}





bool calibCoreAlgorithm::StereoCalibFishEye(calibCoreAlgorithm &camL, calibCoreAlgorithm &camR, RT &stereo_RT) {
    //stereoCalibrate
    std::vector<std::vector<cv::Point3d>> calib_board_corners;

    std::vector<std::vector<Point2d>> image_L_pnts;

    std::vector<std::vector<Point2d>> image_R_pnts;

    cv::Mat R, T, E, F;

    for (size_t num_L = 0; num_L < camL._pair_corner_pnt2ds_image_vec.size(); num_L++) {
        for (size_t num_R = 0; num_R < camR._pair_corner_pnt2ds_image_vec.size(); num_R++) {
            if (camL._pair_corner_pnt2ds_image_vec[num_L].first == camR._pair_corner_pnt2ds_image_vec[num_R].first) {
                image_L_pnts.push_back(camL._pair_corner_pnt2ds_image_vec[num_L].second);
                image_R_pnts.push_back(camR._pair_corner_pnt2ds_image_vec[num_R].second);
                calib_board_corners.push_back(camL._calibrationData._plane3dPntsVec[num_L]);
            } else {
                continue;
            }
        }
    }
    std::cout << "image_L_pnts.size== " << image_L_pnts.size() << endl
              << "image_R_pnts== " << image_R_pnts.size() << endl;

    std::cout << "calib_board_corners.size== " << calib_board_corners.size() << endl;

    double stereo_rms = fisheye::stereoCalibrate(calib_board_corners, image_L_pnts, image_R_pnts,
                                                 camL._fishEyeCamPara._cameraMatrix, camL._fishEyeCamPara._DistortCoeff,
                                                 camR._fishEyeCamPara._cameraMatrix, camR._fishEyeCamPara._DistortCoeff,
                                                 camL._calibrationData._imageSize, R, T,
                                                 fisheye::CALIB_FIX_INTRINSIC,
                                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                                                                  1e-6));

    std::cout << "stereo rms == " << stereo_rms << std::endl;

    stereo_RT._R = R;
    stereo_RT._T = T;
    stereo_RT._stero_rms = stereo_rms;
    std::cout << "stereo_R== " << stereo_RT._R << endl;
    std::cout << "stereo_T== " << stereo_RT._T << endl;

    //yong.qi added
    double disBase = norm(stereo_RT._T, CV_L2);
    std::cout << "disBase== " << disBase << endl << endl;
    stereo_RT._length_T = disBase;
    //yong.qi ended

    //save stero cam calibFiles

    if (!calibCoreAlgorithm::WriteStereoCalibFilesFishEye(camL._resultPath, camL._fishEyeCamPara,camL._serial_num,
                                                          camR._fishEyeCamPara,camR._serial_num, stereo_RT)) {
        return false;
    }
    std::cout << "calib process success... " << std::endl;

    return true;

}

 bool calibCoreAlgorithm::StereoCalibFishEyeROI(calibCoreAlgorithm &camL,calibCoreAlgorithm &camR,RT &stereo_RT)
{
    //stereoCalibrate
    std::vector<std::vector<cv::Point3d>> calib_board_corners;

    std::vector<std::vector<Point2d>> image_L_pnts;

    std::vector<std::vector<Point2d>> image_R_pnts;

    for (size_t num_L = 0; num_L < camL._pair_corner_pnt2ds_image_vec.size(); num_L++) {
        for (size_t num_R = 0; num_R < camR._pair_corner_pnt2ds_image_vec.size(); num_R++) {
            if (camL._pair_corner_pnt2ds_image_vec[num_L].first == camR._pair_corner_pnt2ds_image_vec[num_R].first) {
                image_L_pnts.push_back(camL._pair_corner_pnt2ds_image_vec[num_L].second);
                image_R_pnts.push_back(camR._pair_corner_pnt2ds_image_vec[num_R].second);
                calib_board_corners.push_back(camL._calibrationData._plane3dPntsVec[num_L]);
            } else {
                continue;
            }
        }
    }

    //cout for test
    std::cout << "image_L_pnts.size()== " << image_L_pnts.size() << endl
              << "image_R_pnts.size()== " << image_R_pnts.size() << endl;
    std::cout<<"image_size== "<< camL._calibrationData._imageSize<<endl;
    std::cout << "calib_board_corners.size== " << calib_board_corners.size() << endl;
    std::cout <<"camL._fishEyeCamPara._cameraMatrix== "<<camL._fishEyeCamPara._cameraMatrix<<endl;
    std::cout<<"camL._fishEyeCamPara._DistortCoeff== "<<camL._fishEyeCamPara._DistortCoeff<<endl;
    std::cout <<"camR._fishEyeCamPara._cameraMatrix== "<<camR._fishEyeCamPara._cameraMatrix<<endl;
    std::cout<<"camR._fishEyeCamPara._DistortCoeff== "<<camR._fishEyeCamPara._DistortCoeff<<endl;
    std::cout<<"camL._stereo_RT._R== "<<camL._stereo_RT._R<<endl;
    std::cout<<"camL._stereo_RT._T== "<<camL._stereo_RT._T<<endl;
    //yong.qi ended

//    double stereo_rms = fisheye::stereoCalibrate(calib_board_corners, image_L_pnts, image_R_pnts,
//                                                 camL._fishEyeCamPara._cameraMatrix, camL._fishEyeCamPara._DistortCoeff,
//                                                 camR._fishEyeCamPara._cameraMatrix, camR._fishEyeCamPara._DistortCoeff,
//                                                 camL._calibrationData._imageSize, camL._stereo_RT._R, camL._stereo_RT._T,
//                                                 fisheye::CALIB_USE_INTRINSIC_GUESS,
//                                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
//                                                                  1e-6));


    cv::Mat R,T;
    double stereo_rms = fisheye::stereoCalibrate(calib_board_corners, image_L_pnts, image_R_pnts,
                                                 camL._fishEyeCamPara._cameraMatrix, camL._fishEyeCamPara._DistortCoeff,
                                                 camR._fishEyeCamPara._cameraMatrix, camR._fishEyeCamPara._DistortCoeff,
                                                 camL._calibrationData._imageSize,R, T,
                                                 fisheye::CALIB_FIX_INTRINSIC,
                                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                                                                  1e-6));

    std::cout << "stereo rms == " << stereo_rms << std::endl;

    stereo_RT._R = R;
    stereo_RT._T = T;
    stereo_RT._stero_rms = stereo_rms;
    std::cout << "stereo_R== " << stereo_RT._R << endl;
    std::cout << "stereo_T== " << stereo_RT._T << endl;

    //yong.qi added
    double disBase = norm(stereo_RT._T, CV_L2);
    std::cout << "disBase== " << disBase << endl << endl;
    stereo_RT._length_T = disBase;
    //yong.qi ended

    //save stero cam calibFiles

    if (!calibCoreAlgorithm::WriteStereoCalibFilesFishEye(camL._resultPath, camL._fishEyeCamPara,camL._serial_num,
                                                          camR._fishEyeCamPara,camR._serial_num, stereo_RT)) {
        return false;
    }
    std::cout << "calib process success... " << std::endl;

    return true;

}






void calibCoreAlgorithm::undistortImg(const cv::Mat &src, const cv::Mat &dst, const cv::Mat &cameraMatrix,
                                      const cv::Mat &distCoeffs, const cv::Mat &R) {
    Mat map1, map2;
    Mat new_cameraMatrix;
    cameraMatrix.copyTo(new_cameraMatrix);

    if (dst.rows < 1) {
        return;
    }
    //need to continue to optimizing
    initUndistortRectifyMap(cameraMatrix, distCoeffs, R, new_cameraMatrix, dst.size(), CV_32FC1, map1, map2);
    remap(src, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void calibCoreAlgorithm::ecovacs_initUndistortRectifyMap(cv::Mat _cameraMatrix, cv::Mat _distCoeffs,
                                                         cv::Mat _matR, cv::Mat _newCameraMatrix,
                                                         Size size, int m1type, cv::Mat &_map1, cv::Mat &_map2) {
    Mat cameraMatrix = _cameraMatrix.clone(), distCoeffs = _distCoeffs.clone();
    Mat matR = _matR.clone(), newCameraMatrix = _newCameraMatrix.clone();

    if (m1type <= 0)
        m1type = CV_16SC2;
    CV_Assert(m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2);
    _map1.create(size, m1type);

    Mat map1 = _map1.clone(), map2;

    //yong.qi added
    Mat map1_new = _map1.clone(), map2_new;
    //declare a mat to store mat_kr
    cv::Mat mat_kr = Mat::zeros(map1.size(), CV_64FC1);
    //yong.qi ended
    if (m1type != CV_32FC2) {
        _map2.create(size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1);
        map2 = _map2.clone();

        //yong.qi added;
        map2_new = _map2.clone();
        //yong.qi ended
    } else
        _map2.release();

    Mat_<double> R = Mat_<double>::eye(3, 3);
    Mat_<double> A = Mat_<double>(cameraMatrix), Ar;

    if (!newCameraMatrix.empty())
        Ar = Mat_<double>(newCameraMatrix);
    else
        Ar = getDefaultNewCameraMatrix(A, size, true);

    if (!matR.empty())
        R = Mat_<double>(matR);

    if (!distCoeffs.empty())
        distCoeffs = Mat_<double>(distCoeffs);
    else {
        distCoeffs.create(14, 1, CV_64F);
        distCoeffs = 0.;
    }

    CV_Assert(A.size() == Size(3, 3) && A.size() == R.size());
    CV_Assert(Ar.size() == Size(3, 3) || Ar.size() == Size(4, 3));
    Mat_<double> iR = (Ar.colRange(0, 3) * R).inv(DECOMP_LU); //LU·decomposition
    const double *ir = &iR(0, 0);
    double u0 = A(0, 2), v0 = A(1, 2);
    double fx = A(0, 0), fy = A(1, 1);

    CV_Assert(distCoeffs.size() == Size(1, 4) || distCoeffs.size() == Size(4, 1) ||
              distCoeffs.size() == Size(1, 5) || distCoeffs.size() == Size(5, 1) ||
              distCoeffs.size() == Size(1, 8) || distCoeffs.size() == Size(8, 1) ||
              distCoeffs.size() == Size(1, 12) || distCoeffs.size() == Size(12, 1) ||
              distCoeffs.size() == Size(1, 14) || distCoeffs.size() == Size(14, 1));

    if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
        distCoeffs = distCoeffs.t();

    const double *const distPtr = distCoeffs.ptr<double>();
    double k1 = distPtr[0];
    double k2 = distPtr[1];
    double p1 = distPtr[2];
    double p2 = distPtr[3];
    double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? distPtr[4] : 0.;
    double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[5] : 0.;
    double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[6] : 0.;
    double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[7] : 0.;
    double s1 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[8] : 0.;
    double s2 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[9] : 0.;
    double s3 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[10] : 0.;
    double s4 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[11] : 0.;

    //yong.qi added
    //calculate and save the kr value
    for (int m = 0; m < size.height; m++) {
        double *mkrf = mat_kr.ptr<double>(m);
        double _x = m * ir[1] + ir[2], _y = m * ir[4] + ir[5], _w = m * ir[7] + ir[8];

        for (int n = 0; n < size.width; n++, _x += ir[0], _y += ir[3], _w += ir[6]) {
            double w = 1. / _w, x = _x * w, y = _y * w;
            double x2 = x * x, y2 = y * y;
            double r2 = x2 + y2, _2xy = 2 * x * y;
            double m_kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
            mkrf[n] = (double) m_kr;
        }
    }
    //declare a kr_new to store the corrected kr
    cv::Mat kr_new;
    mat_kr.copyTo(kr_new);

    //calculate the standard deviation of the first line of mat_kr
    cv::Mat mean_value, dev_value;
    Mat row_kr_mat = mat_kr.row(0);
    meanStdDev(row_kr_mat, mean_value, dev_value);
    double std_dev_value = dev_value.at<double>(0, 0);


    double kr_front = 0; //j-1 column value for each row
    double kr_current = 0; //j column value for each row
    double kr_back = 0; //j+1 column value for each row

    //kr threshold
    double thresh_value_delt = 0.1 * std_dev_value;

    //fix kr
    for (int i = 0; i < size.height; i++) {
        double _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];
        for (int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6]) {
            if (j >= 1 && j < (size.width - 1)) //note the range of values,  1<j<cols-1
            {
                kr_front = mat_kr.at<double>(i, j - 1);
                kr_current = mat_kr.at<double>(i, j);
                kr_back = mat_kr.at<double>(i, j + 1);

                double delt_front_current = abs(kr_current - kr_front);
                double delt_back_current = abs(kr_current - kr_back);
                if ((delt_front_current > thresh_value_delt) &&
                    (delt_back_current > thresh_value_delt)) //´Ë´¦&&,ÖØ¸´±È½Ï
                {
                    double m_D = k2 * pow(_w, 2) * pow(_y, 2) + k3 * pow(_y, 4) + k1 * pow(_w, 4);
                    double m_E = k5 * pow(_w, 2) * pow(_y, 2) + k6 * pow(_y, 4) + k4 * pow(_w, 4);
                    double m_F = k2 * pow(_w, 2) + 3 * k3 * pow(_y, 2);
                    double m_G = k5 * pow(_w, 2) + 3 * k6 * pow(_y, 2);
                    double m_H = 2 * k2 * pow(_w, 2) * pow(_y, 2) + 3 * k3 * pow(_y, 4) + k1 * pow(_w, 4);
                    double m_J = 2 * k5 * pow(_w, 2) * pow(_y, 2) + 3 * k6 * pow(_y, 4) + k4 * pow(_w, 4);
                    double X = _x - ir[0];
                    double Xa = _x;
                    double M1 = (k3 * pow(Xa, 6) + m_F * pow(Xa, 4) + m_H * pow(Xa, 2) + pow(_y, 2) * m_D) /
                                (k6 * pow(Xa, 6) + m_G * pow(Xa, 4) + m_J * pow(Xa, 2) + pow(_y, 2) * m_E);
                    double M2 = (k3 * pow(X, 6) + m_F * pow(X, 4) + m_H * pow(X, 2) + pow(_y, 2) * m_D) /
                                (k6 * pow(X, 6) + m_G * pow(X, 4) + m_J * pow(X, 2) + pow(_y, 2) * m_E);
                    double delt_kr = M1 - M2;
                    kr_current = kr_front + delt_kr;
                    kr_new.at<double>(i, j) = kr_current;
                }
            }
        }
    }

    //yong.qi added
    double kr = 0;
    double kr_New = 0;
    for (int i = 0; i < size.height; i++) {
        float *m1f = map1.ptr<float>(i);
        float *m2f = map2.empty() ? 0 : map2.ptr<float>(i);
        short *m1 = (short *) m1f;
        ushort *m2 = (ushort *) m2f;
        double _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];

        for (int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6]) {
            double w = 1. / _w, x = _x * w, y = _y * w;
            double x2 = x * x, y2 = y * y;
            double r2 = x2 + y2, _2xy = 2 * x * y;
            //kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4)*r2);
            kr = kr_new.at<double>(i, j);
            double xd = (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2) + s1 * r2 + s2 * r2 * r2);
            double yd = (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy + s3 * r2 + s4 * r2 * r2);
            cv::Vec3d vecTilt = cv::Vec3d(xd, yd, 1);
            double invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
            double u = fx * invProj * vecTilt(0) + u0;
            double v = fy * invProj * vecTilt(1) + v0;
            if (m1type == CV_16SC2) {
                int iu = saturate_cast<int>(u * INTER_TAB_SIZE);
                int iv = saturate_cast<int>(v * INTER_TAB_SIZE);
                m1[j * 2] = (short) (iu >> INTER_BITS);
                m1[j * 2 + 1] = (short) (iv >> INTER_BITS);
                m2[j] = (ushort) ((iv & (INTER_TAB_SIZE - 1)) * INTER_TAB_SIZE + (iu & (INTER_TAB_SIZE - 1)));
            } else if (m1type == CV_32FC1) {
                m1f[j] = (float) u;
                m2f[j] = (float) v;
            } else {
                m1f[j * 2] = (float) u;
                m1f[j * 2 + 1] = (float) v;
            }
        }
    }
    _map1 = map1;
    _map2 = map2;
}


cv::Mat calibCoreAlgorithm::localUndistortImgPinHole(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef) {


    vector<cv::Point2f> image_points(4);
    std::vector<cv::Point2f> undistort_image_points(4);

    Point2f Point_top_left(0, 0), Point_top_right(img.cols, 0), Point_bottom_left(0, img.rows), Point_bottom_right(
            img.cols, img.rows);
    image_points[0] = Point_top_left;
    image_points[1] = Point_top_right;
    image_points[2] = Point_bottom_left;
    image_points[3] = Point_bottom_right;

    undistortPoints(image_points, undistort_image_points, mK, mDistCoef, noArray(), mK);

//    std::cout << "output mK== " << mK << endl;
//    cout << "\nunPoint_top_left : " << undistort_image_points[0] << endl;
//    cout << "unPoint_top_right : " << undistort_image_points[1] << endl;
//    cout << "unPoint_bottom_left : " << undistort_image_points[2] << endl;
//    cout << "unPoint_bottom_right : " << undistort_image_points[3] << endl;

    int minX = (int) min(undistort_image_points[0].x, undistort_image_points[2].x);
    int maxX = (int) max(undistort_image_points[1].x, undistort_image_points[3].x);

    int minY = (int) min(undistort_image_points[0].y, undistort_image_points[1].y);
    int maxY = (int) max(undistort_image_points[2].y, undistort_image_points[3].y);

    Size original_undistortedImage_size((int) (maxX - minX), (int) (maxY - minY));

    Size new_undistortedImage_size(img.cols, img.rows);


    Rect validPixROI(0, 0, 0, 0);

    double balance_0 = 0, balance_1 = 1;

    vector<Rect> vValidPixROI;

    Mat newCameraMatrix0 = cv::getOptimalNewCameraMatrix(mK, mDistCoef, new_undistortedImage_size, balance_0,
                                                         new_undistortedImage_size,
                                                         &validPixROI, false);
    Mat newCameraMatrix1 = cv::getOptimalNewCameraMatrix(mK, mDistCoef, new_undistortedImage_size, balance_1,
                                                         new_undistortedImage_size,
                                                         &validPixROI, false);

    cv::namedWindow("srcImg_pinhole", 0);
    cv::namedWindow("undistortImg_cameraMatrix_pinhole", 0);
    //cv::namedWindow("undistortImg_newcameraMatrix0_pinhole", 0);
    cv::namedWindow("undistortImg_newcameraMatrix1_pinhole", 0);

    {
        cv::imshow("srcImg_pinhole", img);

        cv::waitKey(100);
    }

    Mat srcundistort;
    {
        cv::Mat mapx, mapy;

        ecovacs_initUndistortRectifyMap(mK, mDistCoef, Mat(), mK, new_undistortedImage_size, CV_32FC1,
                                        mapx, mapy);

        remap(img, srcundistort, mapx, mapy, INTER_AREA);

        cv::imshow("undistortImg_cameraMatrix_pinhole", srcundistort);

        cv::waitKey(100);
    }

//    {
//        cv::Mat mapx, mapy;
//
//       ecovacs_initUndistortRectifyMap(mK, mDistCoef,Mat(),newCameraMatrix0, new_undistortedImage_size, CV_32FC1,
//                                             mapx, mapy);
//
//       remap(img, srcundistort, mapx, mapy, INTER_AREA);
//
//        cv::imshow("undistortImg_newcameraMatrix0_pinhole", srcundistort);
//
//        cv::waitKey(600);
//    }

    {
        cv::Mat mapx, mapy;

        ecovacs_initUndistortRectifyMap(mK, mDistCoef, Mat(), newCameraMatrix1, new_undistortedImage_size, CV_32FC1,
                                        mapx, mapy);

        remap(img, srcundistort, mapx, mapy, INTER_AREA);

        cv::imshow("undistortImg_newcameraMatrix1_pinhole", srcundistort);

        cv::waitKey(100);
    }

    return srcundistort;
}

cv::Mat calibCoreAlgorithm::localUndistortImgFishEye(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef) {


    vector<cv::Point2f> image_points(4);
    std::vector<cv::Point2f> undistort_image_points(4);

    Point2f Point_top_left(0, 0), Point_top_right(img.cols, 0), Point_bottom_left(0, img.rows), Point_bottom_right(
            img.cols, img.rows);
    image_points[0] = Point_top_left;
    image_points[1] = Point_top_right;
    image_points[2] = Point_bottom_left;
    image_points[3] = Point_bottom_right;

    fisheye::undistortPoints(image_points, undistort_image_points, mK, mDistCoef, noArray(), mK);

    int minX = (int) min(undistort_image_points[0].x, undistort_image_points[2].x);
    int maxX = (int) max(undistort_image_points[1].x, undistort_image_points[3].x);

    int minY = (int) min(undistort_image_points[0].y, undistort_image_points[1].y);
    int maxY = (int) max(undistort_image_points[2].y, undistort_image_points[3].y);

    Size original_undistortedImage_size((int) (maxX - minX), (int) (maxY - minY));

    Size new_undistortedImage_size(img.cols, img.rows);

    Size image_size(int(img.cols), int(img.rows));

    Mat newCameraMatrix0, newCameraMatrix1, newCameraMatrix2;
    Rect validPixROI(0, 0, 0, 0);
    double balance_0 = 0, balance_1 = 1;

    vector<Rect> vValidPixROI;

    fisheye::estimateNewCameraMatrixForUndistortRectify(mK, mDistCoef, image_size, noArray(), newCameraMatrix0,
                                                        balance_0, new_undistortedImage_size, 1.0);


    fisheye::estimateNewCameraMatrixForUndistortRectify(mK, mDistCoef, image_size, noArray(), newCameraMatrix1,
                                                        balance_1, new_undistortedImage_size, 1.0);


    cv::namedWindow("srcImg_fisheye", 0);
    cv::namedWindow("undistortImg_cameraMatrix_fisheye", 0);
    //cv::namedWindow("undistortImg_newcameraMatrix0_fisheye", 0);
    cv::namedWindow("undistortImg_newcameraMatrix1_fisheye", 0);
    {
        cv::imshow("srcImg_fisheye", img);
        //std::cout<<"srcImg_fisheye src_mDistCoef== "<<mDistCoef<<endl;
        //std::cout<<"srcImg_fisheye src_mk== "<<mK<<endl<<endl<<endl;

        cv::waitKey(100);
    }

    Mat srcundistort;
    {
        cv::Mat mapx, mapy;

        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), mK, new_undistortedImage_size, CV_32FC1,
                                             mapx, mapy);

        //std::cout<<"undistortImg_cameraMatrix_fisheye mDistCoef== "<<mDistCoef<<endl;

        //std::cout<<"undistortImg_cameraMatrix_fisheye mK== "<<mK<<endl;

        //std::cout<<"undistortImg_cameraMatrix_fisheye new_undistortedImage_size== "<<new_undistortedImage_size<<endl<<endl<<endl;

        remap(img, srcundistort, mapx, mapy, INTER_AREA);

        cv::imshow("undistortImg_cameraMatrix_fisheye", srcundistort);

        cv::waitKey(100);
    }

//    {
//        cv::Mat mapx, mapy;
//
//        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), newCameraMatrix0, new_undistortedImage_size, CV_32FC1,
//                                             mapx, mapy);
//
//        remap(img, srcundistort, mapx, mapy, INTER_AREA);
//
//        cv::imshow("undistortImg_newcameraMatrix0_fisheye", srcundistort);
//
//        cv::waitKey(600);
//    }

//    {
//        cv::Mat mapx, mapy;
//
//        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), newCameraMatrix1, new_undistortedImage_size,
//                                             CV_32FC1,
//                                             mapx, mapy);
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye mDistCoef== "<<mDistCoef<<endl;
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye newCameraMatrix1== "<<newCameraMatrix1<<endl;
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye new_undistortedImage_size== "<<new_undistortedImage_size<<endl<<endl<<endl;
//
//        remap(img, srcundistort, mapx, mapy, INTER_AREA);
//
//        cv::imshow("undistortImg_newcameraMatrix1_fisheye", srcundistort);
//
//        cv::waitKey(100);
//    }

    return srcundistort;
}


void calibCoreAlgorithm::verifyCalibrationPinHoleOK(std::vector<cv::Mat> imgSrcVec, PinholeCamPara &camPara,
                                                    std::vector<cv::Mat> &imgUndistortVec) {
    std::cout << "get in verifyCalibrationOK()" << std::endl;

    if (imgSrcVec.size() < 2) {
        std::cerr << "the image input for verifyCalibration is less than needed!" << std::endl;
        return;
    }

    cv::Mat cameraMatrix = camPara._cameraMatrix;
    cv::Mat distCoeffMat = camPara._distortMatrix;


    //cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 366.68518269, 0, 388.432066,
    //	0, 366.838874604, 228.623216473, 0, 0, 1);

    //cv::Mat distCoeffMat = (cv::Mat_<double>(1, 5) << -0.02724966413904360, 0.0138797293, -0.0144329909, 0.002203856, 0);


    imgUndistortVec.clear();
    for (int i = 0; i < imgSrcVec.size(); ++i) {
        cv::Mat undistortImg = calibCoreAlgorithm::localUndistortImgPinHole(imgSrcVec[i], cameraMatrix, distCoeffMat);
        imgUndistortVec.push_back(undistortImg);
    }

    //yong.qi added for saving to local
    int image_num = 0;
    for (int h = 0; h < imgUndistortVec.size(); ++h) {
        image_num++;
        if (imgUndistortVec[h].channels() == 3) {
            cv::cvtColor(imgUndistortVec[h], imgUndistortVec[h], CV_BGR2GRAY);
        }

        if (_camera._cameraL) {
            char imageName[50];
            string nameTemL = "../data/undistortImg/L/undistortNewMatrix_";
            sprintf(imageName, "%s%d%s", nameTemL.data(), image_num, ".png");
            cv::imwrite(imageName, imgUndistortVec[h]);
        }

        if (_camera._cameraR) {
            char imageName[50];
            string nameTemR = "../data/undistortImg/R/undistortNewMatrix_";
            sprintf(imageName, "%s%d%s", nameTemR.data(), image_num, ".png");
            cv::imwrite(imageName, imgUndistortVec[h]);
        }
    }
    //yong.qi ended

}

void calibCoreAlgorithm::verifyCalibrationFishEyeOK(std::vector<cv::Mat> imgSrcVec, FishEyeCamPara &camPara,
                                                    std::vector<cv::Mat> &imgUndistortVec) {
    std::cout << "get in verifyCalibrationOK()" << std::endl;

    if (imgSrcVec.size() < 1) {
        std::cerr << "the image input for verifyCalibration is less than needed!" << std::endl;
        return;
    }

    cv::Mat cameraMatrix = camPara._cameraMatrix;
    cv::Mat distCoeffMat = camPara._DistortCoeff;

    imgUndistortVec.clear();

    for (int i = 0; i < imgSrcVec.size(); ++i) {
            cv::Mat undistortImg = calibCoreAlgorithm::localUndistortImgFishEye(imgSrcVec[i], cameraMatrix, distCoeffMat);
            imgUndistortVec.push_back(undistortImg);
    }

    //yong.qi added for saving to local
    int image_num = 0;
    for (int h = 0; h < imgUndistortVec.size(); ++h) {
        image_num++;
        if (imgUndistortVec[h].channels() == 3) {
            cv::cvtColor(imgUndistortVec[h], imgUndistortVec[h], CV_BGR2GRAY);
        }

        if (_camera._cameraL) {
            char imageName[50];
            string nameTemL = "../data/undistortImg/L/balance_zero";
            sprintf(imageName, "%s%d%s", nameTemL.data(), image_num, ".png");
            cv::imwrite(imageName, imgUndistortVec[h]);
        }

        if (_camera._cameraR) {
            char imageName[50];
            string nameTemR = "../data/undistortImg/R/balance_zero";
            sprintf(imageName, "%s%d%s", nameTemR.data(), image_num, ".png");
            cv::imwrite(imageName, imgUndistortVec[h]);
        }
    }

}


bool calibCoreAlgorithm::detectCornersPinhole(const cv::Mat imageInput, cv::Size board_size,
                                              std::vector<cv::Point2f> &image_points) {


    std::cout << "board_size== " << board_size << endl;

    cv::Mat imageGray;
    if (imageInput.channels() == 1) {
        imageInput.copyTo(imageGray);
    } else {
        cvtColor(imageInput, imageGray, CV_BGR2GRAY);
    }

    //	//add blur
    cv::GaussianBlur(imageGray, imageGray, cv::Size(3, 3), 0);

    //	//add adaptive threshold
    //	int block_size = 101;
    //	int c_threshold = 10;
    //	Mat thresholdImg;
    //	adaptiveThreshold(imageGray, thresholdImg, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, block_size,
    //		c_threshold);


    if (!findCirclesGrid(imageGray, board_size, image_points, CALIB_CB_ASYMMETRIC_GRID)) {
        std::cerr << "can not find Asymmetric CirclesGrid corners!" << std::endl;
        return false;
    }

    //yong.qi added for test
    cv::Mat colorImg;
    cv::cvtColor(imageGray, colorImg, CV_GRAY2BGR);
    for (int k = 0; k < image_points.size(); ++k) {
        std::string s = std::to_string(k + 1);
        cv::putText(colorImg, s, image_points[k], cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
    }
    //added fort test
    char imageName[50];
    if (_camera._cameraL) {
        string nameTemL = "./data/detectedL/imageDectedL";
        sprintf(imageName, "%s%d%s", nameTemL.data(), _image_count, ".bmp");
        cv::imwrite(imageName, colorImg);
    }

    if (_camera._cameraR) {
        string nameTemR = "./data/detectedR/imageDectedR";
        sprintf(imageName, "%s%d%s", nameTemR.data(), _image_count, ".bmp");
        cv::imwrite(imageName, colorImg);
    }

    //yong.qi ended

    return true;
}

bool calibCoreAlgorithm::detectCornersFishEye(const cv::Mat imageInput, cv::Size board_size,
                                              std::vector<cv::Point2d> &image_points) {
    std::cout << "board_size== " << board_size << endl;

    cv::Mat imageGray;
    if (imageInput.channels() == 1) {
        imageInput.copyTo(imageGray);
    } else {
        cvtColor(imageInput, imageGray, CV_BGR2GRAY);
    }

    //	//add blur
    cv::GaussianBlur(imageGray, imageGray, cv::Size(3, 3), 0);

    //	//add adaptive threshold
    //	int block_size = 101;
    //	int c_threshold = 10;
    //	Mat thresholdImg;
    //	adaptiveThreshold(imageGray, thresholdImg, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, block_size,
    //		c_threshold);


    if (!findCirclesGrid(imageGray, board_size, image_points, CALIB_CB_ASYMMETRIC_GRID)) {
        std::cerr << "can not find Asymmetric CirclesGrid corners!" << std::endl;
        return false;
    }

    //yong.qi added for test
    cv::Mat colorImg;
    cv::cvtColor(imageGray, colorImg, CV_GRAY2BGR);
    for (int k = 0; k < image_points.size(); ++k) {
        std::string s = std::to_string(k + 1);
        cv::putText(colorImg, s, image_points[k], cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
    }
    //added fort test
    char imageName[50];
    if (_camera._cameraL) {
        string nameTemL = "../data/detectedL/imageDectedL";
        sprintf(imageName, "%s%d%s", nameTemL.data(), _image_count, ".bmp");
        cv::imwrite(imageName, colorImg);
    }

    if (_camera._cameraR) {
        string nameTemR = "../data/detectedR/imageDectedR";
        sprintf(imageName, "%s%d%s", nameTemR.data(), _image_count, ".bmp");
        cv::imwrite(imageName, colorImg);
    }
    //ended

    return true;
}

bool calibCoreAlgorithm::detectCornersFishEyeROI(const cv::Mat imageInput, cv::Rect maskRoi, cv::Size board_size,
                                                 std::vector<cv::Point2d> &image_points) {
    image_points.clear();
    if (maskRoi.x < 0 || maskRoi.y < 0) {
        maskRoi.x = 0;
        maskRoi.y = 0;
    }

    if (maskRoi.x > imageInput.cols || maskRoi.y > imageInput.rows) {
        maskRoi.x = imageInput.cols - maskRoi.width;
        maskRoi.y = imageInput.rows - maskRoi.height;
    }

    if (((maskRoi.x + maskRoi.width) > imageInput.cols) || ((maskRoi.y + maskRoi.height) > imageInput.rows)) {
        maskRoi.width = imageInput.cols - maskRoi.x;
        maskRoi.height = imageInput.rows - maskRoi.y;
    }

    cv::Mat imageMaskRoi = cv::Mat(imageInput, maskRoi);

    namedWindow("imageMaskRoi",0);
    cv::imshow("imageMaskRoi",imageMaskRoi);
    cv::waitKey(100);

    if (!findCirclesGrid(imageMaskRoi, board_size, image_points, CALIB_CB_ASYMMETRIC_GRID)) {
        std::cerr << "can not find Asymmetric CirclesGrid corners!" << std::endl;
        return false;
    }
    for (size_t corner_count = 0; corner_count < image_points.size(); corner_count++) {
        image_points[corner_count].x += maskRoi.x;
        image_points[corner_count].y += maskRoi.y;
    }

    //draw for show
    std::vector<cv::Point2f> selectedCorners;
    cv::Point2f corner;
    corner = cv::Point2f(maskRoi.x, maskRoi.y);
    selectedCorners.push_back(corner);
    corner = cv::Point2f(maskRoi.x + maskRoi.width, maskRoi.y);
    selectedCorners.push_back(corner);
    corner = cv::Point2f(maskRoi.x + maskRoi.width, maskRoi.y + maskRoi.height);
    selectedCorners.push_back(corner);
    corner = cv::Point2f(maskRoi.x, maskRoi.y + maskRoi.height);
    selectedCorners.push_back(corner);


    //yong.qi added for test
    cv::namedWindow("corner_detected", 0);
    cv::Mat colorImg;
    cv::cvtColor(imageInput, colorImg, CV_GRAY2BGR);

    //draw maskRoi top and bottom corners
    circle(colorImg, selectedCorners[0], 2, Scalar(0, 0, 255), -1);
    circle(colorImg, selectedCorners[1], 2, Scalar(0, 255, 255), -1);
    circle(colorImg, selectedCorners[2], 2, Scalar(255, 0, 255), -1);
    circle(colorImg, selectedCorners[1], 2, Scalar(255, 255, 0), -1);
    //draw roi contour line
    DrawLine(colorImg, selectedCorners[0], selectedCorners[1], 0,255,255);
    DrawLine(colorImg, selectedCorners[1], selectedCorners[2], 0,255,255);
    DrawLine(colorImg, selectedCorners[2], selectedCorners[3], 0,255,255);
    DrawLine(colorImg, selectedCorners[0], selectedCorners[3], 0,255,255);


    for (int k = 0; k < image_points.size(); ++k) {
        std::string s = std::to_string(k + 1);
        cv::putText(colorImg, s, image_points[k], cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
    }
    cv::imshow("corner_detected", colorImg);
    cv::waitKey(100);
//    //added fort test
//    char imageName[50];
//    if (_camera._cameraL) {
//        string nameTemL = "../data/detectedL/imageDectedL";
//        sprintf(imageName, "%s%d%s", nameTemL.data(), _image_count, ".bmp");
//        cv::imwrite(imageName, colorImg);
//    }
//
//    if (_camera._cameraR) {
//        string nameTemR = "../data/detectedR/imageDectedR";
//        sprintf(imageName, "%s%d%s", nameTemR.data(), _image_count, ".bmp");
//        cv::imwrite(imageName, colorImg);
//    }
//    //ended

    return true;
}

void calibCoreAlgorithm::createROI(const cv::Mat image, int num_roi, std::vector<cv::Rect> &MaskRoiVec) {
    if (num_roi < 8) {
        std::cerr << "the caliboard image is less than 9,please check" << endl;
        return;
    }

    cv::Mat colorImg;

    if(image.channels()==1)
    {
        cvtColor(image,colorImg,CV_GRAY2BGR);
    } else{
        image.copyTo(colorImg);
    }


    MaskRoiVec.clear();

    vector<int> color_num;
    int color;

    cv::Rect maskRoi;
    for (int i = 1; i <= num_roi; ++i) {
        maskRoi.width = image.cols / 3+20;
        maskRoi.height = image.rows / 3+20;

        if (1 == i) {
            maskRoi.x = 20;
            maskRoi.y = 20;
            MaskRoiVec.push_back(maskRoi);
            color=20;
            color_num.push_back(color);
        }

        if (2 == i) {
            maskRoi.x = image.cols / 3+20;
            maskRoi.y = 20;
            MaskRoiVec.push_back(maskRoi);
            color=60;
            color_num.push_back(color);
        }

        if (3 == i) {
            maskRoi.x = image.cols * 2 / 3-20;
            maskRoi.y = 20;
            MaskRoiVec.push_back(maskRoi);
            color=100;
            color_num.push_back(color);
        }

        if (4 == i) {
            maskRoi.x = 10;
            maskRoi.y = image.rows / 3-20;
            MaskRoiVec.push_back(maskRoi);
            color=130;
            color_num.push_back(color);
        }

        if (5 == i) {
            maskRoi.x = image.cols / 3;
            maskRoi.y = image.rows / 3;
            MaskRoiVec.push_back(maskRoi);
            color=180;
            color_num.push_back(color);
        }

        if (6 == i) {
            maskRoi.x = image.cols * 2 / 3-20;
            maskRoi.y = image.rows * 1 / 3-20;
            MaskRoiVec.push_back(maskRoi);
            color=130;
            color_num.push_back(color);
        }

        if (7 == i) {
            maskRoi.x = 20;
            maskRoi.y = image.rows *2/3-20;
            MaskRoiVec.push_back(maskRoi);
            color=200;
            color_num.push_back(color);
        }

        if (8 == i) {
            maskRoi.x = image.cols / 3-20;
            maskRoi.y = image.rows *2/ 3-20;
            MaskRoiVec.push_back(maskRoi);
            color=160;
            color_num.push_back(color);
        }

        if (9 == i) {
            maskRoi.x = image.cols *2/3 -20;
            maskRoi.y = image.rows *2/3-20;
            MaskRoiVec.push_back(maskRoi);
            color=200;
            color_num.push_back(color);
        }
    }


    //yong.qi added for test
    cv::namedWindow("image_drawROI", 0);
    for (int j = 0; j < 9; ++j)
    {
        std::string roi_num_str=std::to_string(j+1);
        cv::putText(colorImg,roi_num_str,cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y),cv::FONT_HERSHEY_PLAIN, 3, Scalar(0, 0, 255));

        //draw for show
        std::vector<cv::Point2f> selectedCorners;
        cv::Point2f corner;
        corner = cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y);
        selectedCorners.push_back(corner);
        corner = cv::Point2f(MaskRoiVec[j].x + MaskRoiVec[j].width, MaskRoiVec[j].y);
        selectedCorners.push_back(corner);
        corner = cv::Point2f(MaskRoiVec[j].x + MaskRoiVec[j].width, MaskRoiVec[j].y+MaskRoiVec[j].height);
        selectedCorners.push_back(corner);
        corner = cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y + MaskRoiVec[j].height);
        selectedCorners.push_back(corner);


        //draw maskRoi top and bottom corners
        circle(colorImg, selectedCorners[0], 2, Scalar(0, 0, 255), -1);
        circle(colorImg, selectedCorners[1], 2, Scalar(0, 255, 255), -1);
        circle(colorImg, selectedCorners[2], 2, Scalar(255, 0, 255), -1);
        circle(colorImg, selectedCorners[1], 2, Scalar(255, 255, 0), -1);
        //draw roi contour line
        DrawLine(colorImg, selectedCorners[0], selectedCorners[1], color_num[j]-10,color_num[j]+10,color_num[j]+55);
        DrawLine(colorImg, selectedCorners[1], selectedCorners[2], color_num[j]-10,color_num[j]+10,color_num[j]+55);
        DrawLine(colorImg, selectedCorners[2], selectedCorners[3], color_num[j]-10,color_num[j]+10,color_num[j]+55);
        DrawLine(colorImg, selectedCorners[0], selectedCorners[3], color_num[j]-10,color_num[j]+10,color_num[j]+55);

        cv::imshow("image_drawROI",colorImg);
        cv::waitKey(100);
    }

}

void calibCoreAlgorithm::createROI_test(const cv::Mat image,int num_roi, std::vector<cv::Rect> &MaskRoiVec)
{

        if (num_roi < num_roi) {
            std::cerr << "the caliboard image is less than needed,please check" << endl;
            return;
        }

        cv::Mat colorImg;

        if(image.channels()==1)
        {
            cvtColor(image,colorImg,CV_GRAY2BGR);
        } else{
            image.copyTo(colorImg);
        }


        MaskRoiVec.clear();

        vector<int> color_num;
        int color;

        cv::Rect maskRoi;
        for (int i = 1; i <= num_roi; ++i) {

            if (1 == i) {
                maskRoi.x = 20;
                maskRoi.y = 20;
                maskRoi.width=30;
                maskRoi.height=30;
                MaskRoiVec.push_back(maskRoi);
                color=100;
                color_num.push_back(color);
            }

            if (2 == i) {
                maskRoi.x = 60;
                maskRoi.y = 60;
                maskRoi.width=image.cols;
                maskRoi.height=image.rows;
                MaskRoiVec.push_back(maskRoi);
                color=200;
                color_num.push_back(color);
            }

        }

        //yong.qi added for test
        cv::namedWindow("image_drawROI", 0);
        for (int j = 0; j < 2; ++j)
        {
            std::string roi_num_str=std::to_string(j+1);
            cv::putText(colorImg,roi_num_str,cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y),cv::FONT_HERSHEY_PLAIN, 3, Scalar(0, 0, 255));

            //draw for show
            std::vector<cv::Point2f> selectedCorners;
            cv::Point2f corner;
            corner = cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y);
            selectedCorners.push_back(corner);
            corner = cv::Point2f(MaskRoiVec[j].x + MaskRoiVec[j].width, MaskRoiVec[j].y);
            selectedCorners.push_back(corner);
            corner = cv::Point2f(MaskRoiVec[j].x + MaskRoiVec[j].width, MaskRoiVec[j].y+MaskRoiVec[j].height);
            selectedCorners.push_back(corner);
            corner = cv::Point2f(MaskRoiVec[j].x, MaskRoiVec[j].y + MaskRoiVec[j].height);
            selectedCorners.push_back(corner);

            //draw maskRoi top and bottom corners
            circle(colorImg, selectedCorners[0], 2, Scalar(0, 0, 255), -1);
            circle(colorImg, selectedCorners[1], 2, Scalar(0, 255, 255), -1);
            circle(colorImg, selectedCorners[2], 2, Scalar(255, 0, 255), -1);
            circle(colorImg, selectedCorners[1], 2, Scalar(255, 255, 0), -1);
            //draw roi contour line
            DrawLine(colorImg, selectedCorners[0], selectedCorners[1], color_num[j]-10,color_num[j]+10,color_num[j]+55);
            DrawLine(colorImg, selectedCorners[1], selectedCorners[2], color_num[j]-10,color_num[j]+10,color_num[j]+55);
            DrawLine(colorImg, selectedCorners[2], selectedCorners[3], color_num[j]-10,color_num[j]+10,color_num[j]+55);
            DrawLine(colorImg, selectedCorners[0], selectedCorners[3], color_num[j]-10,color_num[j]+10,color_num[j]+55);

            cv::imshow("image_drawROI",colorImg);
            cv::waitKey(100);
        }

}



void calibCoreAlgorithm::DrawLine(Mat &img, Point start, Point end, int colorScalar_B,int colorScalar_G,int colorScalar_R) {

    int thickness = 2;
    int lineType = 8;
    line(img,
         start,
         end,
         Scalar(colorScalar_B, colorScalar_G, colorScalar_R),
         thickness,
         lineType
    );

}


bool calibCoreAlgorithm::generateObjPntsInCalibBoard(const int num_img_corner_detected, const double square_size,
                                                     const cv::Size board_size,
                                                     std::vector<std::vector<cv::Point3f>> &corner_in_caliboard,
                                                     std::vector<std::vector<cv::Point2f>> &corner_in_caliboard2f) {
    std::cout << "square_size== " << square_size << endl;

    std::cout << "board_size.height== " << board_size.height;
    std::cout << "board_size.width== " << board_size.width;

    corner_in_caliboard.clear();
    vector<Point3f> point3fsTem;
    vector<Point2f> point2fsTem;

    for (size_t image_cout = 0; image_cout < num_img_corner_detected; image_cout++) {
        point3fsTem.clear();
        point2fsTem.clear();
        for (int row = 0; row < board_size.height; ++row) {
            for (int col = 0; col < board_size.width; ++col) {
                Point3f pntTem3f;
                Point2f pntTem2f;

                pntTem3f.x = (2 * col + row % 2) * square_size;
                pntTem3f.y = (row) * square_size;
                pntTem3f.z = 0;

                pntTem2f = Point2f(pntTem3f.x, pntTem3f.y);

                point3fsTem.push_back(pntTem3f);
                point2fsTem.push_back(pntTem2f);
            }
        }
        corner_in_caliboard.push_back(point3fsTem);
        corner_in_caliboard2f.push_back(point2fsTem);
    }
    return true;
}

bool calibCoreAlgorithm::generateObjPntsInCalibBoardFishEye(const int num_img_corner_detected, const double square_size,
                                                            const cv::Size board_size,
                                                            std::vector<std::vector<cv::Point3d>> &corner_in_caliboard3d,
                                                            std::vector<std::vector<cv::Point2d>> &corner_in_caliboard2d) {
    std::cout << "square_size== " << square_size << endl;

    std::cout << "board_size.height== " << board_size.height;

    std::cout << "board_size.width== " << board_size.width;

    corner_in_caliboard3d.clear();

    corner_in_caliboard2d.clear();

    vector<Point3d> point3dsTem;

    vector<Point2d> point2dsTem;

    for (size_t image_cout = 0; image_cout < num_img_corner_detected; image_cout++) {
        point3dsTem.clear();
        point2dsTem.clear();
        for (int row = 0; row < board_size.height; ++row) {
            for (int col = 0; col < board_size.width; ++col) {
                Point3d pntTem3d;
                Point2d pntTem2d;

                pntTem3d.x = (2 * col + row % 2) * square_size;
                pntTem3d.y = (row) * square_size;
                pntTem3d.z = 0;

                pntTem2d = Point2d(pntTem3d.x, pntTem3d.y);

                point3dsTem.push_back(pntTem3d);
                point2dsTem.push_back(pntTem2d);
            }
        }
        corner_in_caliboard3d.push_back(point3dsTem);
        corner_in_caliboard2d.push_back(point2dsTem);
    }
    return true;


}


bool calibCoreAlgorithm::computeReprojectionError(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                                  const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                                  const std::vector<std::vector<cv::Point2f>> &image_corner_pnts,
                                                  const std::vector<std::vector<cv::Point3f>> &object_points) {
    std::cout << "get in computeReprojectionError()" << endl;
    vector<vector<Point2f> > error_image_points;
    std::vector<cv::Point2f> perframe_imagepnt;
    for (int k = 0; k < object_points.size(); ++k) {
        perframe_imagepnt.clear();

        projectPoints(object_points[k], rvecsMat[k], tvecsMat[k], cameraMatrix, distCoeffs, perframe_imagepnt);
        error_image_points.push_back(perframe_imagepnt);

        double err = cv::norm(image_corner_pnts[k], perframe_imagepnt, cv::NORM_L2);

    }

    vector<double> absErr;
    vector<Point2f> subDst;
    int totalPoints = 0;
    double totalError = 0;

    for (int m = 0; m < object_points.size(); ++m) {

        vector<Point2f> temp;
        cv::subtract(error_image_points[m], image_corner_pnts[m], temp);
        copy(temp.begin(), temp.end(), back_inserter(subDst));
        double err;
        int n = image_corner_pnts[m].size();
        err = norm(error_image_points[m], image_corner_pnts[m], CV_L2);
        absErr.push_back((float) sqrt(err * err / n));
        totalError += err * err;
        totalPoints += n;
    }
    totalError = std::sqrt(totalError / totalPoints);

    Scalar xmean, ymean;
    Scalar xstd_dev, ystd_dev;
    Scalar errormean, errorstd_dev;
    meanStdDev(subDst, errormean, errorstd_dev);

    _pinholeCamPara._ReprojectionError[0] = errorstd_dev[0];
    _pinholeCamPara._ReprojectionError[1] = errorstd_dev[1];
    _pinholeCamPara._totalReproNormErr = totalError;

    return true;

}

bool calibCoreAlgorithm::computeReprojectionErrorFishEye(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                                         const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                                         const std::vector<std::vector<cv::Point2d>> &image_corner_pnts,
                                                         const std::vector<std::vector<cv::Point3d>> &object_points) {
    std::cout << "get in computeReprojectionError()" << endl;
    vector<vector<Point2d> > error_image_points;
    std::vector<cv::Point2d> perframe_imagepnt;
    for (int k = 0; k < object_points.size(); ++k) {
        perframe_imagepnt.clear();

        fisheye::projectPoints(object_points[k], perframe_imagepnt, rvecsMat[k], tvecsMat[k], cameraMatrix, distCoeffs);
        error_image_points.push_back(perframe_imagepnt);

        double err = cv::norm(image_corner_pnts[k], perframe_imagepnt, cv::NORM_L2);
        //std::cout << "err== " << err << endl;

    }

    vector<double> absErr;
    vector<Point2d> subDst;
    int totalPoints = 0;
    double totalError = 0;

    for (int m = 0; m < object_points.size(); ++m) {

        vector<Point2d> temp;
        cv::subtract(error_image_points[m], image_corner_pnts[m], temp);
        copy(temp.begin(), temp.end(), back_inserter(subDst));
        double err;
        int n = image_corner_pnts[m].size();
        err = norm(error_image_points[m], image_corner_pnts[m], CV_L2);
        absErr.push_back((float) sqrt(err * err / n));
        totalError += err * err;
        totalPoints += n;
    }
    totalError = std::sqrt(totalError / totalPoints);

    Scalar xmean, ymean;
    Scalar xstd_dev, ystd_dev;
    Scalar errormean, errorstd_dev;
    meanStdDev(subDst, errormean, errorstd_dev);

    _fishEyeCamPara._ReprojectionError[0] = errorstd_dev[0];
    _fishEyeCamPara._ReprojectionError[1] = errorstd_dev[1];
    _fishEyeCamPara._totalReproNormErr = totalError;

    return true;

}


bool calibCoreAlgorithm::WriteSingleCalibFilesPinHole(const std::string &calib_file_name,
                                                      const PinholeCamPara &camCalibPara) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    fs << "cam_matrix" << camCalibPara._cameraMatrix;
    fs << "cam_distortion" << camCalibPara._distortMatrix;
    fs << "totalReproError" << camCalibPara._totalReproNormErr;
    fs << "ReproError_X" << camCalibPara._ReprojectionError[0];
    fs << "ReproError_Y" << camCalibPara._ReprojectionError[1];
    fs.release();
    return true;
}

bool calibCoreAlgorithm::ReadSingleCalibFilesPinHole(const std::string &calib_file_name, PinholeCamPara &camCalibPara) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    fs["cam_matrix"] >> camCalibPara._cameraMatrix;
    fs["cam_distortion"] >> camCalibPara._distortMatrix;
    fs["totalReproError"] >> camCalibPara._totalReproNormErr;
    fs["ReproError_X"] >> camCalibPara._ReprojectionError[0];
    fs["ReproError_Y"] >> camCalibPara._ReprojectionError[1];
    return true;
}


//stereo calibFiles wrtie
bool calibCoreAlgorithm::WriteStereoCalibFilesPinHole(const std::string &calib_file_name, const PinholeCamPara &camL,const string serial_num_L,
                                                      const PinholeCamPara &camR,const string serial_num_R,
                                                      const RT &stereo_RT) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    //Left Cam calibfiles

    fs<<"serial_num_L"<<serial_num_L;
    fs << "cam_matrix_L" << camL._cameraMatrix;
    fs << "cam_distortion_L" << camL._distortMatrix;
    fs << "totalReproError_L" << camL._totalReproNormErr;
    fs << "ReproError_X_L" << camL._ReprojectionError[0];
    fs << "ReproError_Y_L" << camL._ReprojectionError[1];


    //Right Cam calib
    fs<<"serial_num_R"<<serial_num_R;
    fs << "cam_matrix_R" << camR._cameraMatrix;
    fs << "cam_distortion_R" << camR._distortMatrix;
    fs << "totalReproError_R" << camR._totalReproNormErr;
    fs << "ReproError_X_R" << camR._ReprojectionError[0];
    fs << "ReproError_Y_R" << camR._ReprojectionError[1];

    //stereo cam Files
    fs<<"right_to_left"<<" ";
    fs << "system_rotation" << stereo_RT._R;
    fs << "system_translation" << stereo_RT._T;
    fs << "stereo_calib_rms" << stereo_RT._stero_rms;
    fs << "stereo_length_base" << stereo_RT._length_T;
    fs.release();
    return true;
}

bool calibCoreAlgorithm::WriteStereoCalibFilesFishEye(const std::string &calib_file_name, const FishEyeCamPara &camL,const string serial_num_L,
                                                      const FishEyeCamPara &camR,const string serial_num_R,
                                                      const RT &stereo_RT) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    //Left Cam calibfiles

    fs<<"serial_num_L"<<serial_num_L;
    fs << "cam_matrix_L" << camL._cameraMatrix;
    fs << "cam_distortion_L" << camL._DistortCoeff;
    fs << "totalReproError_L" << camL._totalReproNormErr;
    fs << "ReproError_X_L" << camL._ReprojectionError[0];
    fs << "ReproError_Y_L" << camL._ReprojectionError[1];


    //Right Cam calib
    fs<<"serial_num_R"<<serial_num_R;
    fs << "cam_matrix_R" << camR._cameraMatrix;
    fs << "cam_distortion_R" << camR._DistortCoeff;
    fs << "totalReproError_R" << camR._totalReproNormErr;
    fs << "ReproError_X_R" << camR._ReprojectionError[0];
    fs << "ReproError_Y_R" << camR._ReprojectionError[1];

    //stereo cam Files
    fs<<"right_to_left"<<" ";
    fs << "system_rotation" << stereo_RT._R;
    fs << "system_translation" << stereo_RT._T;
    fs << "stereo_calib_rms" << stereo_RT._stero_rms;
    fs << "stereo_length_base" << stereo_RT._length_T;
    fs.release();
    return true;
}

bool calibCoreAlgorithm::WriteCaliboard2CamPose(const std::string &caliboard_to_cam_pose_name,
                                                const std::vector<cv::Mat> tvecsMat,

                                                const std::vector<cv::Mat> rvecsMat) {

    if (tvecsMat.size() < 1 || rvecsMat.size() < 1) {
        std::cerr << "the tvecsMat or rvecsMat size is less than needed" << std::endl;
    }

    cv::FileStorage fs(caliboard_to_cam_pose_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }

    for (int i = 0; i < rvecsMat.size(); ++i) {
        fs << "imgCount" << i;
        fs << "R" << rvecsMat[i];
        fs << "T" << tvecsMat[i];
    }

    fs.release();

    return true;
}


bool calibCoreAlgorithm::ReadStereoCalibFilesPinHole(const std::string &calib_file_name, PinholeCamPara &camL,
                                                     PinholeCamPara &camR,
                                                     RT &stereo_RT) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    //Left Cam calibfiles
    fs["cam_matrix_L"] >> camL._cameraMatrix;
    fs["cam_distortion_L"] >> camL._distortMatrix;
    fs["totalReproError_L"] >> camL._totalReproNormErr;
    fs["ReproError_X_L"] >> camL._ReprojectionError[0];
    fs["ReproError_Y_L"] >> camL._ReprojectionError[1];


    //Right Cam calib
    fs["cam_matrix_R"] >> camR._cameraMatrix;
    fs["cam_distortion_R"] >> camR._distortMatrix;
    fs["totalReproError_R"] >> camR._totalReproNormErr;
    fs["ReproError_X_R"] >> camR._ReprojectionError[0];
    fs["ReproError_Y_R"] >> camR._ReprojectionError[1];


    //stereo cam Files
    fs["system_rotation"] >> stereo_RT._R;
    fs["system_translation"] >> stereo_RT._T;
    fs["stereo_calib_rms"] >> stereo_RT._stero_rms;
    fs.release();
}

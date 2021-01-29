## 本标定程序主要包括

1. 鱼眼镜头的标定；
2. 针孔模型的标定；
3. 提供初值的鱼眼镜头的标定；
4. 提供初值的针孔镜头的标定；

标定板：OpenCV提供的4*11格，圆心间距根据不同标定板设定。

图片文件夹下，存放一个imagePath.txt，里面为每张图片的路径：例如

../data/test_data/20190903/L/ 3.png
../data/test_data/20190903/L/ 4.png
../data/test_data/20190903/L/ 5.png
../data/test_data/20190903/L/ 6.png

鱼眼模型的解读：[一分钟详解鱼眼镜头标定基本原理及实现](https://mp.weixin.qq.com/s/VyxoTaYtYPB-Bfh3JCXl1A)                


如果在立体标定后，还需要立体校正，则可以参考如下代码：

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
    
    //rectifyImgs
    cv::Mat Rl,Rr,Pl,Pr,Q;
    cv::fisheye::stereoRectify(camLeft._fishEyeCamPara._cameraMatrix,camLeft._fishEyeCamPara._DistortCoeff,
            camRight._fishEyeCamPara._cameraMatrix,camRight._fishEyeCamPara._DistortCoeff,
            camLeft._calibrationData._imageSize,stero_RT._R,stero_RT._T,Rl,Rr,Pl,Pr,Q,0,camLeft._calibrationData._imageSize,0,1.0);



    //based on the R,P calculated above to calculate mapx,mapy
    cv::Mat mapLx,mapLy,mapRx,mapRy;
    cv::fisheye::initUndistortRectifyMap(camLeft._fishEyeCamPara._cameraMatrix,camLeft._fishEyeCamPara._DistortCoeff,
                            Rl,Pl, camLeft._calibrationData._imageSize,CV_32FC1,mapLx,mapLy);
    cv::fisheye::initUndistortRectifyMap(camRight._fishEyeCamPara._cameraMatrix,camRight._fishEyeCamPara._DistortCoeff,
                            Rr,Pr, camRight._calibrationData._imageSize,CV_32FC1,mapRx,mapRy);
    
    //use the map to rectifyImg
    cv::namedWindow("rectifyImgL",0);
    cv::namedWindow("rectifyImgR",0);
    cv::Mat rectifyImgL,rectifyImgR;
    for (int k = 0; k <camLeft._src_imageVec.size(); ++k)
    {
        cv::Mat imgL=camLeft._src_imageVec[k];
        cv::Mat imgR=camRight._src_imageVec[k];
        remap(imgL,rectifyImgL,mapLx,mapLy,cv::INTER_LINEAR);
        remap(imgR,rectifyImgR,mapRx,mapRy,cv::INTER_LINEAR);
        imshow("rectifyImgL",rectifyImgL);
        imshow("rectifyImgR",rectifyImgR);
        waitKey(6000);
    }

#endif

- [加群](#加群)

<a name="加群"></a>

![](/home/yong/workspace/2_project_files/0_github_upload_files/vision3d/awesome-3D-Vision-Papers/imgs/公众号.jpg)

「3D视觉工坊」技术交流群已经成立，目前大约有12000人，方向主要涉及3D视觉、CV&深度学习、SLAM、三维重建、点云后处理、自动驾驶、CV入门、三维测量、VR/AR、3D人脸识别、医疗影像、缺陷检测、行人重识别、目标跟踪、视觉产品落地、视觉竞赛、车牌识别、硬件选型、学术交流、求职交流、ORB-SLAM系列源码交流、深度估计等。工坊致力于干货输出，不做搬运工，为计算机视觉领域贡献自己的力量！欢迎大家一起交流成长~

添加小助手微信：CV_LAB，备注学校/公司+姓名+研究方向即可加入工坊一起学习进步。

![](/home/yong/workspace/2_project_files/0_github_upload_files/vision3d/awesome-3D-Vision-Papers/imgs/微信.jpg)

QQ群「3D视觉工坊」，群号：949193717

![](/home/yong/workspace/2_project_files/0_github_upload_files/vision3d/awesome-3D-Vision-Papers/imgs/QQ群.jpg)

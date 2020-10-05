#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class ConfigParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConfigParam(std::string configfile,int sys_type);

    double _testDiscardTime;

    static Eigen::Matrix4d GetEigTbc();
    static cv::Mat GetMatTbc();
    static Eigen::Matrix4d GetEigT_cb();
    static cv::Mat GetMatT_cb();
    static int GetLocalWindowSize();
    static double GetImageDelayToIMU();
    static bool GetAccMultiply9p8();

    static double GetG(){return _g;}

    std::string _bagfile;
    std::string _imageTopic;
    std::string _imuTopic;
    std::string _imageTopic0;
    std::string _imageTopic1;
    std::string _imageRGBTopic;
    std::string _depthTopic;


    static std::string getTmpFilePath();
    static std::string _tmpFilePath;

    static double GetVINSInitTime(){return _nVINSInitTime;}
    static bool GetRealTimeFlag() {return _bRealTime;}

private:
    static Eigen::Matrix4d _EigTbc;
    static cv::Mat _MatTbc;
    static Eigen::Matrix4d _EigTcb;
    static cv::Mat _MatTcb;
    static int _LocalWindowSize;
    static double _ImageDelayToIMU;
    static bool _bAccMultiply9p8;

    static double _g;
    static double _nVINSInitTime;
    static bool _bRealTime;
};

}

#endif // CONFIGPARAM_H

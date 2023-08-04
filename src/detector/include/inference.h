#include <iterator>
#include <memory>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
using namespace InferenceEngine;
static constexpr int INPUT_W = 416;    // Width of input
static constexpr int INPUT_H = 416;    // Height of input
static constexpr int ARMOR_NUM_CLASSES = 8;  // Number of classes
static constexpr int ARMOR_NUM_COLORS = 4;   // Number of color
static constexpr int TOPK = 128;       // TopK
static constexpr float ARMOR_NMS_THRESH = 0.3;
static constexpr float ARMOR_BBOX_CONF_THRESH = 0.75;
static constexpr float ARMOR_MERGE_CONF_ERROR = 0.15;
static constexpr float ARMOR_MERGE_MIN_IOU = 0.9;
static constexpr int BUFF_NUM_CLASSES = 2;  // Number of classes
static constexpr int BUFF_NUM_COLORS = 2;   // Number of color
static constexpr float BUFF_NMS_THRESH  = 0.1;
static constexpr float BUFF_BBOX_CONF_THRESH = 0.6;
static constexpr float BUFF_MERGE_CONF_ERROR = 0.15;
static constexpr float BUFF_MERGE_MIN_IOU = 0.2;

struct ArmorObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};


class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();
    bool detect(cv::Mat &src,std::vector<ArmorObject>& objects);
    bool initModel(std::string path);
private:

    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network;                // 网络
    InferenceEngine::ExecutableNetwork executable_network;       // 可执行网络
    InferenceEngine::InferRequest infer_request;      // 推理请求
    InferenceEngine::MemoryBlob::CPtr moutput;
    std::string input_name;
    std::string output_name;
    
    Eigen::Matrix<float,3,3> transfrom_matrix;
};

struct BuffObject
{
    cv::Point2f apex[5];
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

class BuffDetector
{

private:
    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network;                // 网络
    InferenceEngine::ExecutableNetwork executable_network;       // 可执行网络
    InferenceEngine::InferRequest infer_request;      // 推理请求
    InferenceEngine::MemoryBlob::CPtr moutput;
    std::string input_name;
    std::string output_name;
    
    Eigen::Matrix<float,3,3> transfrom_matrix;

public:
    BuffDetector();
    ~BuffDetector();

    bool detect(cv::Mat &src,std::vector<BuffObject>& objects);
    bool initModel(std::string path);

};
struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};
float calcTriangleArea(cv::Point2f pts[3]);
float calcTetragonArea(cv::Point2f pts[4]);
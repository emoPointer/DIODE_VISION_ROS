#include "./DxImageProc.h"
#include "./GxIAPI.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
class DaHengCamera
{
private:
    //枚举操作成功与失败信息
    GX_STATUS status = GX_STATUS_SUCCESS;
    //相机设备
    GX_DEV_HANDLE hDevice = NULL;
    //定 义 GXDQBuf 的 传 入 参 数,包含图片内存，大小等信息
    PGX_FRAME_BUFFER pFrameBuffer;

    bool                set_color;
    int64_t             lastImgTimestamp;           ///< timestamp of last img
    void*               pGammaLut;                  ///< Gamma look up table
    int                 m_nContrastLutLength;       ///< Contrast look up table length
    int64_t             nColorCorrectionParam;

public:
    //构造函数，初始化库
    DaHengCamera();
    //打开设备
    int StartDevice(int serial_number);
    //使设备开始采集
    bool SetStreamOn();
    //设置分辨率，支持1:1(最大1280*1024),1:2,2:1,2:2(最小640*512),默认1:1
    bool SetResolution(int width_scale = 1, int height_scale = 1);
    //手动设置曝光值,单位us,正常大小应在2000至8000
    bool SetExposureTime(int ExposureTime);
    //设置曝光增益
    bool SetGAIN(int value, int ExpGain);
    //采集图像
    cv::Mat GetMat(cv::Mat &Src);
    bool Set_BALANCE(int value, float value_number);
    //析构函数释放资源
    ~DaHengCamera();
};
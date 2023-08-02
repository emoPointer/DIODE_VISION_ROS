#include <img_pub.h>

DaHengCamera::DaHengCamera()
{
    //初始化库
    status = GXInitLib();
    //检测初始化是否成功
    if (status != GX_STATUS_SUCCESS)
        ROS_ERROR("camera init failed!");
    else
        ROS_INFO("camera init success");
}
/**
 * @brief 打开相机
 * @param serial_number为要打开设备的序列号
 * @return 返回检测到的连接相机个数
 */
int DaHengCamera::StartDevice(int serial_number)
{
    uint32_t nDeviceNum = 0;
    //枚 举 设 备 列 表
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (serial_number > nDeviceNum)
    {
        ROS_ERROR("The device number is incorrect. The number exceeds the enumeration");
        return -1;
    }
    //打 开 设 备
    status = GXOpenDeviceByIndex(serial_number, &hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO("camera open success");
        return nDeviceNum;
    }
    else
    {
        ROS_ERROR("camera open failed!");
        return -1;
    }
}
/**
 * @brief DaHengCamera::SetResolution   设置分辨率
 * @param width_scale   宽比例
 * @param height_scale  高比例
 * @return bool 返回是否成功
 */
bool DaHengCamera::SetResolution(int width_scale, int height_scale)
{
    //配 置 一 个 2x2 的 Binning 和 2x2 的 Decimation
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t nBinningH = width_scale;
    int64_t nBinningV = height_scale;
    int64_t nDecimationH = width_scale;
    int64_t nDecimationV = height_scale;

    //设 置 水 平 和 垂 直 Binning 模 式 为 Sum 模 式
    status = GXSetEnum(hDevice, GX_ENUM_BINNING_HORIZONTAL_MODE,
                       GX_BINNING_HORIZONTAL_MODE_AVERAGE);
    status = GXSetEnum(hDevice, GX_ENUM_BINNING_VERTICAL_MODE,
                       GX_BINNING_VERTICAL_MODE_AVERAGE);
    status = GXSetInt(hDevice, GX_INT_BINNING_HORIZONTAL, nBinningH);
    status = GXSetInt(hDevice, GX_INT_BINNING_VERTICAL, nBinningV);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO("Resolution setting succeeded");
        return true;
    }
    else
    {
        ROS_ERROR("Resolution setting failed!");
        return false;
    }
}
bool DaHengCamera::SetStreamOn()
{
    //设置buffer数量
    //设 置 采 集 buffer 个 数
    status = GXSetAcqusitionBufferNumber(hDevice, 2);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO("buffer设置成功!");
    }
    else
    {
        ROS_ERROR("buffer设置失败!");
    }

    status = GXSetBool(hDevice, GX_BOOL_CHUNKMODE_ACTIVE, true);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO( "帧信息模式已设置为使能!");
    }
    else
    {
        ROS_ERROR( "帧信息模式设置失败!");
    }

    status = status = GXSetEnum(hDevice, GX_ENUM_CHUNK_SELECTOR, GX_CHUNK_SELECTOR_CHUNK_TIME_STAMP);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO( "时间戳帧信息已启用!");
    }
    else
    {
        ROS_ERROR( "时间戳帧信息启用失败!");
    }
    //开 采
    status = GXStreamOn(hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO( "开始采集图像!");
        return true;
    }
    else
    {
        ROS_ERROR( "采集失败!");
        return false;
    }
}
/**
 * @brief DaHengCamera::SetGAIN 手动设置曝光增益
 * @param value 选择曝光增益通道 0-B,1-G,2-R,3-All
 * @param ExpGain   具体增益值 范围0-16
 * @return
 */
bool DaHengCamera::SetGAIN(int value, int ExpGain)
{
    if (value == 0)
    {
        //选 择 增 益 通 道 类 型
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
    }
    else if (value == 1)
    {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
    }
    else if (value == 2)
    {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
    }
    else
    {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    }
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, ExpGain);
    if (status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}
/**
 * @brief DaHengCamera::Set_BALANCE 手动白平衡,设置之前必须先关闭自动白平衡,具有记忆功能
 * @param value 选择平衡通道 0-B,1-G,2-R
 * @param value_number 平衡系数
 * @return
 */
bool DaHengCamera::Set_BALANCE(int value, float value_number)
{
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    if (value == 0)
    {
        //选 择 白 平 衡 通 道
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    }
    else if (value == 1)
    {
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    }
    else
    {
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    }
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, (float)value_number);
    if (status == GX_STATUS_SUCCESS){
        ROS_INFO( "白平衡设置成功");
        return true;
    }
    else
    {
        ROS_ERROR( "白平衡设置失败");
        return false;
    }
}
/**
 * @brief DaHengCamera::GetMat 读取图像
 * @param Src 引入方式传递
 * @return bool 返回是否成功
 */
cv::Mat DaHengCamera::GetMat(cv::Mat &Src)
{
    // ------------------------------------------- For Stream------------------------------------------------------------
    //调 用 GXDQBuf 取 一 帧 图 像
    status = GXDQBuf(hDevice, &pFrameBuffer, 1000);
    if (status == GX_STATUS_SUCCESS && pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        lastImgTimestamp = pFrameBuffer->nTimestamp;
        char *pRGB24Buf = new char[pFrameBuffer->nWidth * pFrameBuffer->nHeight * 3]; //输 出 图 像 RGB 数 据
        if (pRGB24Buf == NULL)
        {
            // return false;
        }
        else
        {
            memset(pRGB24Buf, 0, pFrameBuffer->nWidth * pFrameBuffer->nHeight * 3 * sizeof(char));
            //缓 冲 区 初 始 化
        }
        DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR3; //选 择 插 值 算 法
        DX_PIXEL_COLOR_FILTER nBayerType = DX_PIXEL_COLOR_FILTER(BAYERBG);
        //选 择 图 像 Bayer 格 式
        bool bFlip = false;

        VxInt32 DxStatus = DxRaw8toRGB24(pFrameBuffer->pImgBuf, pRGB24Buf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, cvtype, nBayerType, bFlip);
        if (DxStatus != DX_OK)
        {
            ROS_ERROR("Raw8 to RGB24 failed!");
            if (pRGB24Buf != NULL)
            {
                delete[] pRGB24Buf;
                pRGB24Buf = NULL;
            }
            // return false;
        }
        if (set_color)
        {
            DxStatus = DxImageImprovment(pRGB24Buf, pRGB24Buf,pFrameBuffer->nWidth, pFrameBuffer->nHeight, nColorCorrectionParam,NULL,pGammaLut);
            if (DxStatus != DX_OK)
                ROS_ERROR( "Color Set Failed!");
        }

        cv::Mat src = cv::Mat(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);
        memcpy(src.data, pRGB24Buf, pFrameBuffer->nWidth * pFrameBuffer->nHeight * 3);
        // flip(src, src, -1);
        src.copyTo(Src);

        delete[] pRGB24Buf;
        pRGB24Buf = NULL;
        // //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
        status = GXQBuf(hDevice, pFrameBuffer);
        return src;
    }
    else
    {
        // cout << "读取图片缓冲失败" << endl;
        ROS_ERROR("读取图片缓冲失败");
        status = GXQBuf(hDevice, pFrameBuffer);
        // return false;
    }
}
bool DaHengCamera::SetExposureTime(int ExposureTime)
{
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, ExposureTime);
    if (status == GX_STATUS_SUCCESS)
    {
        ROS_INFO("曝光值设置成功");
        return true;
    }
    else
    {
        ROS_ERROR( "曝光值设置失败");
        return false;
    }
}
/**
 * @brief DaHengCamera::~DaHengCamera 析构函数关闭设备
 */
DaHengCamera::~DaHengCamera()
{
    //停 采
    status = GXStreamOff(hDevice);
    //关闭设备链接
    status = GXCloseDevice(hDevice);
    //释放库
    status = GXCloseLib();
    ROS_INFO( "析构!");
}
// image_transport::Publisher image_pub; // 图像发布者
// void imageCallback(DaHengCamera Daheng)
// {
//     try
//     {
//         cv::Mat image_raw;
        
//         auto DaHeng_stauts = Daheng.GetMat(image_raw);
//         // 将处理后的图像转换为ROS图像消息并发布
//         sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
//         image_pub.publish(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
// }
int main(int argc, char** argv)
{   
    setlocale(LC_ALL,"");
    DaHengCamera DaHeng;
    DaHeng.StartDevice(1);
        // 设置分辨率
    DaHeng.SetResolution(1,1);
        // 开始采集帧
    DaHeng.SetStreamOn();
        //更新时间戳，设置时间戳偏移量
        // DaHeng.UpdateTimestampOffset(time_start);
        // 设置曝光事件
    DaHeng.SetExposureTime(3500);
        // 设置1
    DaHeng.SetGAIN(3, 16);
        // manual白平衡 BGR->012
    DaHeng.Set_BALANCE(0,1.56);
    DaHeng.Set_BALANCE(1,1.0);
    DaHeng.Set_BALANCE(2,1.548);   
    cv::Mat IMG;
    
    ros::init(argc, argv, "img_pub"); // 初始化ROS节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("camera/image_raw", 1); // 创建图像发布者
    

    ros::Rate loop_rate(100); // 发布频率为100Hz
    while (nh.ok())
    {
        cv::Mat original_image=DaHeng.GetMat(IMG);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", IMG).toImageMsg(); // 转换为ROS图像消息
        image_pub.publish(msg); // 发布图像消息
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
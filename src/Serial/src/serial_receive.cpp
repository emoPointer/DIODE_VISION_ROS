#include <serial_receive.h>
/**
 * @brief 解算角速度数据
 * 
 * @param data 角速度首地址指针
 * @return
 */
bool c_board::getGyro(unsigned char *data)
{    
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    gyro[0] = exchange_data(f1,process_float_data[16]);
    gyro[1] = exchange_data(f2,process_float_data[20]);
    gyro[2] = exchange_data(f3,process_float_data[24]);
    return true;
}
bool c_board::getQuat(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];
    unsigned char* f4 = &data[12];

    quat[0] = exchange_data(f1,process_float_data[0]);
    quat[1] = exchange_data(f2,process_float_data[4]);
    quat[2] = exchange_data(f3,process_float_data[8]);
    quat[3] = exchange_data(f4,process_float_data[12]);
    return true;
}

/**
 * @brief 解算加速度数据
 * 
 * @param data 加速度首地址指针
 * @return
 */
bool c_board::getAcc(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    acc[0] = exchange_data(f1,process_float_data[28]);
    acc[1] = exchange_data(f2,process_float_data[32]);
    acc[2] = exchange_data(f3,process_float_data[36]);
    return true;
}
bool c_board::getSpeed(unsigned char *data)
{
    unsigned char* f1 = &data[0];

    speed = exchange_data(f1,process_float_data[40]);

//    fmt::print(fmt::fg(fmt::color::white), "speed: {} \n", bullet_speed);
    return true;
}
/**
 * @brief 将4个uchar转换为float
 * @param data data首地址指针
 * @return
 */
float c_board::exchange_data(unsigned char *data,float need_return_float_data)
{
    *((uint8_t *)&need_return_float_data) = data[0];
    *(((uint8_t *)&need_return_float_data+1)) = data[1];
    *(((uint8_t *)&need_return_float_data+2)) = data[2];
    *(((uint8_t *)&need_return_float_data+3)) = data[3];
    return need_return_float_data;
};

/**************************************
Date: May 11, 2023
功能: CRC发送校验中间函数
输入参数： 
***************************************/
uint16_t CRC_INIT = 0xffff;
const unsigned char CRC8_INIT = 0xff;
uint16_t c_board::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;

	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}

	while (dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}

	return wCRC;
}
unsigned char c_board::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
	unsigned char ucIndex;

	while (dwLength--)
	{
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}

	return (ucCRC8);
}
/**************************************
Date: May 11, 2023
功能: CRC发送校验
输入参数： 
***************************************/
void c_board::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wCRC = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}

	wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
void c_board::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return;

	ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
	pchMessage[dwLength - 1] = ucCRC;
}
/**************************************
Date: May 11, 2023
功能: CRC接收校验
输入参数： 
***************************************/
unsigned int c_board::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return 0;

	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
	return (ucExpected == pchMessage[dwLength - 1]);
}
uint32_t c_board::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return 0;
	}

	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
c_board::c_board(){     //在构造函数中引入外参值
    ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
    private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyUSB0"); //Fixed serial port number //固定串口号
    private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200);
    serial_receicer_pub = n.advertise<rm_msgs::serial_receiver>("serial_receive", 1);
    ROS_INFO_STREAM("Data ready");
    try
    { 
        //Attempts to initialize and open the serial port //尝试初始化与开启串口
        Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
        Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
        Stm32_Serial.setTimeout(_time);
        ROS_INFO_STREAM("Stm32_open_success");
        Stm32_Serial.open(); //Open the serial port //开启串口
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("robot can not open Stm32_serial port,Please check the serial port cable!: " << e.what()); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
    }
    if(Stm32_Serial.isOpen())
    {
        ROS_INFO_STREAM("robot Stm32_serial port opened"); //Serial port opened successfully //串口开启成功提示
    }
}
c_board::~c_board(){
    Stm32_Serial.close(); //Close the serial port //关闭串口  
    ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
}
bool c_board::Get_serial_Data()
{
  Stm32_Serial.read(rdata,1);
  if(rdata[0]==0XA5){
  Stm32_Serial.read(rdata+1,31);
  Stm32_Serial.read(rdata+32,18);
    // printf("0x%02X ", (unsigned char)rdata[0]);
    //cout<<hex<<rdata[0];
    //  printf("0X%x ",rdata[0]);
    // printf("0X%x ",rdata[59]);
    if (rdata[0]==0XA5 && Verify_CRC8_Check_Sum(rdata, 3) && Verify_CRC16_Check_Sum(rdata,50))
    {
        mode = rdata[1];
        getQuat(&rdata[3]);
        getGyro(&rdata[19]);
        getAcc(&rdata[31]);
        getSpeed(&rdata[43]);
        Verify_CRC16_Check_Sum(rdata,50);
    }else 
      return false;
  }
}
void c_board::pub_serial_receiver()
{
    rm_msgs::serial_receiver msg;
    msg.header = "c_board";
    msg.quat[0] = quat[0];
    msg.quat[1] = quat[1];
    msg.quat[2] = quat[2];
    msg.quat[3] = quat[3];
    msg.gyro[0] = gyro[0];
    msg.gyro[1] = gyro[1];
    msg.gyro[2] = gyro[2];
    msg.acc[0] = acc[0];
    msg.acc[1] = acc[1];
    msg.acc[2] = acc[2];
    msg.speed = speed;
    msg.mode = mode;
    serial_receicer_pub.publish(msg);
}
int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "serial_receice"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  c_board number4; //Instantiate an object //实例化一个对象
  ros::Rate loop_rate(100); // 发布频率为100Hz
  while (ros::ok())
  {
    if (true == number4.Get_serial_Data())
        number4.pub_serial_receiver();
        
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;  
} 

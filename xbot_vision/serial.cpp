#include "serial.h"
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

//初始化
int _serial::setup()
{
	if (wiringPiSetup() < 0)//wiringPi初始化
	{
		return 1;
	}
	if ((this->fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)//初始化串口，波特率115200
	{
		return 1;
	}
}

//判断是否接收到指令，若是，改变发送状态
void _serial::is_open()
{
	if (serialDataAvail(this->fd) >= 1)    //判断串口接收缓存中是否有数据
	{
		this->Rx_dat0[0] = serialGetchar(this->fd);//将缓存数据存入接收数组
		if (this->Rx_dat0[0] == 0xa1)   //若接受到32发送的 数据 0xa1,改变发送状态
		{
			if (this->is_RX_on == 0)
			{
				this->is_RX_on = 1;
				this->Rx_dat0[0] = 0x00;
			}
			else if (this->is_RX_on == 1)
			{
				this->is_RX_on = 0;
				this->Rx_dat0[0] = 0x00;
			}
		}
	}
	serialFlush(this->fd);//清除串口接收缓存
}

//将数字整数与小数分离并分别赋给数组对应位置，{待处理数据，符号，整数，小数}
void _serial::separate_num(double num, int *num_0, int *num_1, int *num_2)
{
	if (num >= 0)//符号赋值
	{
		*num_0 = 0;
	}
	else if (num < 0)
	{
		*num_0 = 1;
		num *= -1;
	}

	double num_2d;//待处理小数部分
	*num_1 = (int)num;//整数赋值
	num_2d = num - (int)num;
	for (int t = 0; num_2d - (int)num_2d >= 1e-10 && t < 2; t++)//保留两位小数并去除小数点
	{
		num_2d *= 10;
	}
	*num_2 = (int)num_2d;//小数赋值
}

//数据处理并发送数据
void _serial::send_vision_target()
{
	if (pit_yaw[3].x <= 10 && pit_yaw[3].x >= -10)//对偏航角限幅并处理
	{
		this->separate_num(pit_yaw[3].x, &this->Tx_art[1], &this->Tx_art[2], &this->Tx_art[3]);
		yaw_ref = 0;
	}
	if (pit_yaw[3].y <= 10 && pit_yaw[3].y >= -10)//对俯仰角限幅并处理
	{
		this->separate_num(pit_yaw[3].y, &this->Tx_art[4], &this->Tx_art[5], &this->Tx_art[6]);
		pit_ref = 0;
	}

	int i = 0;
	for (; i < 8; i++)//发送数据
	{
		serialPutchar(this->fd, this->Tx_art[i]);
	}
}

//判断发送状态，若是，发送数据
void _serial::is_send()
{
	if (this->is_RX_on == 1)
	{
		this->send_vision_target();
	}
}

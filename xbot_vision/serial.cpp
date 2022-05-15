#include "serial.h"
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

//��ʼ��
int _serial::setup()
{
	if (wiringPiSetup() < 0)//wiringPi��ʼ��
	{
		return 1;
	}
	if ((this->fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)//��ʼ�����ڣ�������115200
	{
		return 1;
	}
}

//�ж��Ƿ���յ�ָ����ǣ��ı䷢��״̬
void _serial::is_open()
{
	if (serialDataAvail(this->fd) >= 1)    //�жϴ��ڽ��ջ������Ƿ�������
	{
		this->Rx_dat0[0] = serialGetchar(this->fd);//���������ݴ����������
		if (this->Rx_dat0[0] == 0xa1)   //�����ܵ�32���͵� ���� 0xa1,�ı䷢��״̬
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
	serialFlush(this->fd);//������ڽ��ջ���
}

//������������С�����벢�ֱ𸳸������Ӧλ�ã�{���������ݣ����ţ�������С��}
void _serial::separate_num(double num, int *num_0, int *num_1, int *num_2)
{
	if (num >= 0)//���Ÿ�ֵ
	{
		*num_0 = 0;
	}
	else if (num < 0)
	{
		*num_0 = 1;
		num *= -1;
	}

	double num_2d;//������С������
	*num_1 = (int)num;//������ֵ
	num_2d = num - (int)num;
	for (int t = 0; num_2d - (int)num_2d >= 1e-10 && t < 2; t++)//������λС����ȥ��С����
	{
		num_2d *= 10;
	}
	*num_2 = (int)num_2d;//С����ֵ
}

//���ݴ�����������
void _serial::send_vision_target()
{
	if (pit_yaw[3].x <= 10 && pit_yaw[3].x >= -10)//��ƫ�����޷�������
	{
		this->separate_num(pit_yaw[3].x, &this->Tx_art[1], &this->Tx_art[2], &this->Tx_art[3]);
		yaw_ref = 0;
	}
	if (pit_yaw[3].y <= 10 && pit_yaw[3].y >= -10)//�Ը������޷�������
	{
		this->separate_num(pit_yaw[3].y, &this->Tx_art[4], &this->Tx_art[5], &this->Tx_art[6]);
		pit_ref = 0;
	}

	int i = 0;
	for (; i < 8; i++)//��������
	{
		serialPutchar(this->fd, this->Tx_art[i]);
	}
}

//�жϷ���״̬�����ǣ���������
void _serial::is_send()
{
	if (this->is_RX_on == 1)
	{
		this->send_vision_target();
	}
}

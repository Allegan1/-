#pragma once

class _serial
{
public:

	_serial() {};
	~_serial() {};

	int Rx_dat0[1] = { 0x00 };//接收数组，0xA1为发送的检验数据
	int is_RX_on = 0;//发送数据状态变量，0为停止发送，1为开始发送
	int fd;//文件描述符
	int Tx_art[8] = { 0xBF,0,0,0,0,0,0,0xFB };//待发送数据数组，0为开始，1为偏航角符号，2为偏航角整数，3为偏航角小数，4为俯仰角符号，5为俯仰角整数，6为俯仰角小数，7为结束

	//初始化
	int setup();
	
	//判断是否接收到指令，若是，改变发送状态
	void is_open();

	//将数字整数与小数分离并分别赋给数组对应位置，{待处理数据，符号，整数，小数}
	void separate_num(double num, int *num_0, int *num_1, int *num_2);

	//数据处理并发送数据
	void send_vision_target();

	//判断发送状态，若是，发送数据
	void is_send();

};

extern double pit_ref;                     // 俯仰角
extern double yaw_ref;                     // 偏航角
extern Point2f pit_yaw[4]
#pragma once

class _serial
{
public:

	_serial() {};
	~_serial() {};

	int Rx_dat0[1] = { 0x00 };//�������飬0xA1Ϊ���͵ļ�������
	int is_RX_on = 0;//��������״̬������0Ϊֹͣ���ͣ�1Ϊ��ʼ����
	int fd;//�ļ�������
	int Tx_art[8] = { 0xBF,0,0,0,0,0,0,0xFB };//�������������飬0Ϊ��ʼ��1Ϊƫ���Ƿ��ţ�2Ϊƫ����������3Ϊƫ����С����4Ϊ�����Ƿ��ţ�5Ϊ������������6Ϊ������С����7Ϊ����

	//��ʼ��
	int setup();
	
	//�ж��Ƿ���յ�ָ����ǣ��ı䷢��״̬
	void is_open();

	//������������С�����벢�ֱ𸳸������Ӧλ�ã�{���������ݣ����ţ�������С��}
	void separate_num(double num, int *num_0, int *num_1, int *num_2);

	//���ݴ�����������
	void send_vision_target();

	//�жϷ���״̬�����ǣ���������
	void is_send();

};

extern double pit_ref;                     // ������
extern double yaw_ref;                     // ƫ����
extern Point2f pit_yaw[4]
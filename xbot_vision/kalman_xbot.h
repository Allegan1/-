#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include<string>

using namespace cv;
using namespace Eigen;
using namespace std;


class kalman
{
public:
	int statesize;
	int usize;
	int measize;

	VectorXd x;
	MatrixXd A;
	MatrixXd B;
	VectorXd U;

	MatrixXd P;
	MatrixXd Q;

	MatrixXd H;
	MatrixXd R;
	VectorXd Z;

	int if_prepare = 0;//�ж��Ƿ��ʼ��
	float t = 0.066;//ʱ����
	float x1 = 0, x2 = 0;//�������Ԥ�����
	
	Point2f Current_point;//��ǰ��
	Point2f Past_point;//��һʱ�̵�
	Point2f PPast_point;//����ʱ�̵�

	//�ٶ�����ٶ�Ȩֵ��1/t
	float v_x = 15;
	float v_y = 15;
	float a_x = 15;
	float a_y = 15;

	void kalman_init_1(int statesize, int usize, int measize);
	void kalman_init_2();
	void update_Z();
	void update();
	void predict_1();
	Point2f predict_2(Point2f Current_point, Point2f Past_point, Point2f PPast_point);//ֱ�ӵ����������������{��ǰ�㣬��һʱ�̵㣬����ʱ�̵�}�����Ԥ��λ������{x,y}

};

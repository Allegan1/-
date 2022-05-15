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

	int if_prepare = 0;//判断是否初始化
	float t = 0.066;//时间间隔
	float x1 = 0, x2 = 0;//用于输出预测误差
	
	Point2f Current_point;//当前点
	Point2f Past_point;//上一时刻点
	Point2f PPast_point;//上上时刻点

	//速度与加速度权值，1/t
	float v_x = 15;
	float v_y = 15;
	float a_x = 15;
	float a_y = 15;

	void kalman_init_1(int statesize, int usize, int measize);
	void kalman_init_2();
	void update_Z();
	void update();
	void predict_1();
	Point2f predict_2(Point2f Current_point, Point2f Past_point, Point2f PPast_point);//直接调用这个函数，输入{当前点，上一时刻点，上上时刻点}，输出预测位置向量{x,y}

};

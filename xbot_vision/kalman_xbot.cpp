#include"kalman_xbot.h"

//�����С��ʼ��
void kalman::kalman_init_1(int statesize, int usize, int measize)
{
	this->statesize = statesize;
	this->usize = usize;
	this->measize = measize;

	x.resize(statesize);
	x.setZero();

	A.resize(statesize, statesize);
	A.setZero();

	B.resize(statesize, usize);
	B.setZero();

	U.resize(usize);
	U.setZero();

	P.resize(statesize, statesize);
	P.setZero();

	Q.resize(statesize, statesize);
	Q.setZero();

	H.resize(measize, statesize);
	H.setZero();

	R.resize(measize, measize);
	R.setZero();

	Z.resize(measize);
	Z.setZero();
}

//������ֵ��ʼ��
void kalman::kalman_init_2()
{
	if (if_prepare == 0)//ֻ��ʼ��һ��
	{
		kalman_init_1(6, 2, 6);

		// x = x0 + v t + a t^2 / 2
		A <<    1, 0, t, 0, 0.5 * t * t, 0,
				0, 1, 0, t, 0, 0.5 * t * t,
				0, 0, 1, 0, t, 0,
				0, 0, 0, 1, 0, t,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1;

		H.setIdentity();
		P.setIdentity();
		R.setIdentity()*0.5;
		Q.setIdentity();
		Z.setZero();
		B.setZero();
		U.setZero();

		if_prepare = 1;
	}
}

//Z�������£���ֵ��ǰλ�ã��ٶȣ����ٶ�
void kalman::update_Z()
{
	float velocity_x;//�ٶ�
	float velocity_y;
	float acceleration_x;//���ٶ�
	float acceleration_y;

	velocity_x = (Current_point.x - Past_point.x) * v_x;
	velocity_y = (Current_point.y - Past_point.y) * v_y;
	acceleration_x = (Current_point.x + PPast_point.x - 2 * Past_point.x) * a_x;
	acceleration_y = (Current_point.y + PPast_point.y - 2 * Past_point.y) * a_y;

	Z <<    Current_point.x,
			Current_point.y,
			velocity_x,
			velocity_y,
			acceleration_x,
			acceleration_y;
}

//����
void kalman::update()
{
	MatrixXd K, H_T, Bulf1, Bulf2;

	H_T = H.transpose();
	Bulf1 = H * P * H_T + R;
	Bulf2 = Bulf1.inverse();

	K = P * H_T * Bulf2;

	x = x + K * (Z - H * x);
	P = P - K * H * P;
}

//Ԥ��
void kalman::predict_1()
{
	MatrixXd A_T = A.transpose();
	x = A * x + B * U;
	P = A * P * A_T + Q;
}

//ִ�к���������{��ǰ�㣬��һʱ�̵㣬����ʱ�̵�}�����Ԥ��λ������{x,y}
Point2f kalman::predict_2(Point2f Current_point, Point2f Past_point, Point2f PPast_point)
{
	Point2f x_2f = Point2f(0, 0);
	kalman_init_2();//��ʼ��

	this->Current_point = Current_point;//λ�ø���
	this->Past_point = Past_point;
	this->PPast_point = PPast_point;

	update_Z();//����
	update();

	predict_1();//Ԥ��

	cout << "��ǰ��    ( " << Current_point.x << " , " << Current_point.y << ")" << endl;
	cout << "Ԥ���    ( " << x[0] << " , " << x[1] << ")" << endl;
	cout << "Ԥ����� �� "<< x1 - Current_point.x << " , " << x2 - Current_point.y <<" )"<< endl;//���Ԥ�������
	x1 = x[0]; x2 = x[1];
	x_2f = Point2f(x[0], x[1]);
	return x_2f;
}
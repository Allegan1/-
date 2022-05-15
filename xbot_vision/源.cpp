#include <opencv2/opencv.hpp>
#include <iostream> 
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<string>
#include<cmath>
#include<vector>
#include"ArmorDetect.h"
#include"kalman_xbot.h"
#include"serial.h"
using namespace cv;
using namespace std;

/////////////////////////////数据初始化//////////////////////////////////////////
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y-p2.y))
#define real_length 5.6f//装甲板实际长度
#define real_width 6.0f//装甲板实际宽度
Mat frame;
Mat cam_matrix = (Mat_<double>(3, 3) << 1121.61740650421, 0, 981.279558920923,
    0, 1162.98566972283, 534.792218978870,
    0, 0, 1);//内参矩阵
Mat distortion_coeff = (Mat_<double>(5, 1) << -0.0242139775783717, 0.249471547805513, 0, 0, -0.435455759383010);//畸变系数
vector<vector<int >> light_points;//光源经过的点
kalman k1;//创建卡尔曼对象
_serial serial_0;//创建通信对象

double pit_ref;                     // 俯仰角
double yaw_ref;                     // 偏航角
Point2f pit_yaw[4] = {Point2f(0, 0),Point2f(0, 0),Point2f(0, 0),Point2f(0, 0)};	//俯仰角与偏航角，0为当前，1为上一时刻，2为上上时刻，3为预测
 
///////////////////////////////////////////////////////////////////////////////

//进行一个旋转矩阵的画
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness)
{
    cv::Point2f Vertex[4];
    rect.points(Vertex);
    for (int i = 0; i < 4; i++)
    {
        cv::line(img, Vertex[i], Vertex[(i + 1) % 4], color, thickness);
    }
}
//利用相机信息进行测距（投影原理）
void getDistanceDanmu(RotatedRect armor, double& dist, float width_target, float height_target)
{

    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target;
    float fx_h = cam_matrix.at<double>(1, 1) * height_target;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    dist = (dist_w + dist_h) / 2;
}






//图片预处理
void preprocessing(Mat& mask, int i)
{
    if (i == 1)
    {
        inRange(mask, Scalar(11, 43, 250), Scalar(34, 255, 255), mask);
    }
    if (i == 0)
    {

        inRange(mask, Scalar(100, 100, 100), Scalar(124, 255, 255), mask);
    }
    Mat structureElement = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
    /*  Mat structureElement1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
     erode(mask, mask, structureElement1);*/
    dilate(mask, mask, structureElement, Point(-1, -1), 1);

    threshold(mask, mask, 100, 255, THRESH_BINARY);


}


//寻找轮廓
void findcontoursfirst(Mat &mask, std::vector<std::vector<cv::Point>> &contours,std::vector<cv::Vec4i> &hierarchy)
{

   
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    Mat dst3 = Mat::zeros(mask.size(), CV_8UC3);
    RNG rng(12345);//随机颜色
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(dst3, contours, i, color, 2, 8, hierarchy, 0, Point(0, 0));//绘制矩形
    }
    imshow("轮廓", dst3);

}









int main()
{
	if (serial_0.setup() == 1)//串口初始化
	{
		cout << "wiringPi or serial setup error" << endl;
	}

    ArmorInfo armor_info1;//装甲板或符文板信息
    std::vector<std::vector<cv::Point>> light_contours;  // 灯条轮廓
    std::vector<cv::RotatedRect> rot_rect_list;          // 灯条矩形集合
    std::vector<cv::Rect> rect_list;                     // 灯条矩形集合
    std::vector<struct TargetSize> target_size_list;     // 目标尺寸集合(一一对应)
    std::vector<LightInfo> light_list;
    vector<ArmorInfo>Rune;//符文板集合
    vector<ArmorInfo>Rune_target;

    ArmorDetect armor1;//初始化类


    int mood = 0;//模式选择

    //世界坐标系下坐标设置
    float half_x = real_length / 2.0f;
    float half_y = real_width / 2.0f;//数据格式很重要
    std::vector<cv::Point3f> point3d;
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));




    double fps, fps2;
    char string[10], string2[10];
    double t = 0, t2 = 0;//用于计算帧率




    double a = 100, b = 255;
    /*   VideoCapture capture("C:/Users/ALAN/Downloads/装甲板测试视频.avi");*/
    VideoCapture capture(0);
    Mat  img, mask;
    int delay = 30;//用于暂停

    if (!capture.read(frame))
    {
        printf("can not open ...\n");
        return -1;
    }
    namedWindow("output", WINDOW_AUTOSIZE);

    while (capture.read(frame))
    {



        t = (double)cv::getTickCount();
        t2 = (double)cv::getTickCount();


        cvtColor(frame, img, COLOR_BGR2HSV);
        preprocessing(img, mood);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        findcontoursfirst(img, contours, hierarchy);
        
        if (mood == 1)
        {
            
         
            armor1.findlight(contours, light_contours, rot_rect_list, rect_list, target_size_list);

           
            armor1.filterEnemyLight(light_contours, rot_rect_list, rect_list, target_size_list, light_list);

            armor1.findArmor(light_list, frame);
            armor_info1 =armor1.getArmorInfo();
            drawRotatedRect(frame, armor_info1.rrect, cv::Scalar(255, 0, 0), 2);
        }


        if (mood == 0)
        {
           
           
            armor1.findRune(contours, Rune);
            if (Rune.size() != 0)//默认取最右端符文板，当其熄灭后会自动切到下一个
            {

                armor_info1 = Rune[0];
            }

            drawRotatedRect(frame, armor_info1.rrect, cv::Scalar(255, 0, 0), 2);



        }

        t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();//初次计算帧率
        Mat rot_vector, translation_vector;


        if (armor_info1.pos.p.size() >= 4)

        {
            solvePnP(point3d, armor_info1.pos.p, cam_matrix, distortion_coeff, rot_vector, translation_vector); //pnp解算
            double tx = translation_vector.at<double>(0, 0);
            double ty = translation_vector.at<double>(1, 0);
            double tz = translation_vector.at<double>(2, 0);
            double dis = sqrt(tz * tz + ty * ty + tx * tx);//利用solvepnp得出数据进行距离运算

            double tan_pitch = ty / sqrt(tx * tx + tz * tz);//进行俯仰角/偏航角计算
            double tan_yaw = tx / tz;
            pit_ref = -atan(tan_pitch) * 180 / CV_PI;
            yaw_ref = atan(tan_yaw) * 180 / CV_PI;

            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();//第二次计算帧率


            fps2 = 1.0 / t2;//帧率相关
            fps = 1.0 / t;

            sprintf_s(string, "%.2f", fps);      // 帧率保留两位小数
            sprintf_s(string2, "%.2f", fps2);
            std::string fpsString("FPS:");
            std::string fpsString2("FPS2:");
            fpsString += string;                    // 在"FPS:"后加入帧率数值字符串
            fpsString2 += string2;
     
            double dist = 0;
            getDistanceDanmu(armor_info1.rrect, dist, half_x, half_y);//利用投影原理获得距离
            cout << fpsString << endl;
            cout << fpsString2 << endl;
            cout << dis << endl;
            cout << dist << endl;
            cout << "俯仰角" << pit_ref << endl << "偏航角" << yaw_ref  << endl;

			////////////////////////////////////////////////////////////////////////////////////

			pit_yaw[2] = pit_yaw[1];
			pit_yaw[1] = pit_yaw[0];
			pit_yaw[0] = Point2f(pit_ref, yaw_ref);

			pit_yaw[3] = k1.predict_2(pit_yaw[0], pit_yaw[1], pit_yaw[2]);

			//////////////////////////////////////////////////////////////////////////////////////

            int x = armor_info1.rrect.center.x;
            int y = armor_info1.rrect.center.y;
            light_points.push_back({ x,y });



        }

        /////////////////////////////清除数据，返还内存//////////////////////////////////////////
        std::vector<std::vector<cv::Point>>().swap(light_contours);  // 灯条轮廓
        std::vector<cv::RotatedRect>().swap(rot_rect_list);          // 灯条矩形集合
        std::vector<cv::Rect>().swap(rect_list);                     // 灯条矩形集合
        std::vector<struct TargetSize>().swap(target_size_list);     // 目标尺寸集合(一一对应)
        std::vector<LightInfo>().swap(light_list);
        vector<Point2f>().swap(armor_info1.pos.p);
        vector<ArmorInfo>().swap(Rune);
        //////////////////////////////////////////////////////////////////////////////////////





        imshow("output", frame);
     

        

		serial_0.is_open();//判断是否接收到指令，若是，改变发送状态
		serial_0.is_send();//判断发送状态，若是，发送数据

        if (delay >= 0 && waitKey(delay) >= 32)//暂停函数
            waitKey(0);
        else
            waitKey(30);
    }

    return 0;








}
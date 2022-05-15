#pragma once
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<string>
#include<cmath>
#include<vector>

using namespace cv;
using namespace std;

#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y-p2.y))

#ifndef ARMORDETECT
#define ARMORDETECT


struct TargetSize
{

    int shortwidth;
    int len;                               // ��������
    float ratio;                           // �����
    int area;                              // ���
    float area_ratio;                      // �����
    float area_len_ratio;                  // ����ȳ���
    int corners;                           // ����αƽ�
    int slope_offset;                      // �Ƕ�ƫ��
};

struct LightInfo
{
    int idx;                               // ����
    std::vector<cv::Point>contours;        // ����
    struct TargetSize size;                // �����ߴ���Ϣ
    cv::RotatedRect rrect;                 // ��������
    cv::Rect rect;                         // ��������
};

struct QuadrilateralPos
{
    vector<Point2f> p;                      // �����ĸ��ǵ�����
};
struct ArmorInfo
{
    struct QuadrilateralPos pos;           // װ�װ��ĸ���������ĻͶӰ����
    cv::RotatedRect rrect;                 // װ�װ����

}; 







class ArmorDetect
{
public:
	ArmorDetect();
	~ArmorDetect();

    //����ʶ��������������
	void findlight(std::vector<std::vector<cv::Point>> contours,
        std::vector<std::vector<cv::Point>>& target_contours,
        std::vector<cv::RotatedRect>& rot_rect_list, std::vector<cv::Rect>& rect_list, std::vector<TargetSize>& target_size_list);
    //����ɸѡ
    void filterEnemyLight(std::vector<std::vector<cv::Point>> light_contours,
        std::vector<cv::RotatedRect> rot_rect_list,
        std::vector<cv::Rect> rect_list,
        std::vector<struct TargetSize> target_size_list,
        std::vector<LightInfo>& light_info);
	
    //���������ϳ�һ��װ�װ�
    void findArmor(std::vector<LightInfo>& light_list, Mat& frame);
    //������ת����
    RotatedRect adjustRect(const cv::RotatedRect& rect);
    //ͨ����������������ת����
    RotatedRect boundingRRect(const cv::RotatedRect& left, const cv::RotatedRect& right);
    //ƥ����İ�
    void findRune(std::vector<std::vector<cv::Point>> contours, vector<ArmorInfo>& Rune);
    //��װ�װ���нǵ�����
    void getTarget2dPoinstion(const cv::RotatedRect& rect, std::vector<cv::Point2f>& target2d);
    
    //����װ�װ����İ���Ϣ
    ArmorInfo getArmorInfo()
    {
        return armor;
    }

private:
    ArmorInfo armor;//װ�װ����İ���Ϣ

};

ArmorDetect::ArmorDetect()
{
}

ArmorDetect::~ArmorDetect()
{
}





#endif // !ARMORDETECT


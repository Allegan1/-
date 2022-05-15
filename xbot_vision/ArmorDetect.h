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
    int len;                               // 轮廓长度
    float ratio;                           // 长宽比
    int area;                              // 面积
    float area_ratio;                      // 面积比
    float area_len_ratio;                  // 面积比长度
    int corners;                           // 多边形逼近
    int slope_offset;                      // 角度偏移
};

struct LightInfo
{
    int idx;                               // 索引
    std::vector<cv::Point>contours;        // 轮廓
    struct TargetSize size;                // 轮廓尺寸信息
    cv::RotatedRect rrect;                 // 灯条矩形
    cv::Rect rect;                         // 灯条矩形
};

struct QuadrilateralPos
{
    vector<Point2f> p;                      // 矩形四个角点坐标
};
struct ArmorInfo
{
    struct QuadrilateralPos pos;           // 装甲板四个顶点在屏幕投影坐标
    cv::RotatedRect rrect;                 // 装甲板矩形

}; 







class ArmorDetect
{
public:
	ArmorDetect();
	~ArmorDetect();

    //初步识别灯条并过滤噪点
	void findlight(std::vector<std::vector<cv::Point>> contours,
        std::vector<std::vector<cv::Point>>& target_contours,
        std::vector<cv::RotatedRect>& rot_rect_list, std::vector<cv::Rect>& rect_list, std::vector<TargetSize>& target_size_list);
    //二次筛选
    void filterEnemyLight(std::vector<std::vector<cv::Point>> light_contours,
        std::vector<cv::RotatedRect> rot_rect_list,
        std::vector<cv::Rect> rect_list,
        std::vector<struct TargetSize> target_size_list,
        std::vector<LightInfo>& light_info);
	
    //两个灯条合成一个装甲板
    void findArmor(std::vector<LightInfo>& light_list, Mat& frame);
    //调整旋转矩形
    RotatedRect adjustRect(const cv::RotatedRect& rect);
    //通过两个灯条绘制旋转矩形
    RotatedRect boundingRRect(const cv::RotatedRect& left, const cv::RotatedRect& right);
    //匹配符文板
    void findRune(std::vector<std::vector<cv::Point>> contours, vector<ArmorInfo>& Rune);
    //对装甲板进行角点排序
    void getTarget2dPoinstion(const cv::RotatedRect& rect, std::vector<cv::Point2f>& target2d);
    
    //返回装甲板或符文板信息
    ArmorInfo getArmorInfo()
    {
        return armor;
    }

private:
    ArmorInfo armor;//装甲板或符文板信息

};

ArmorDetect::ArmorDetect()
{
}

ArmorDetect::~ArmorDetect()
{
}





#endif // !ARMORDETECT


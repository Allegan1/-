
#include"ArmorDetect.h"

void ArmorDetect::findlight(std::vector<std::vector<cv::Point>> contours,
    std::vector<std::vector<cv::Point>>& target_contours,
    std::vector<cv::RotatedRect>& rot_rect_list, std::vector<cv::Rect>& rect_list, std::vector<TargetSize>& target_size_list)//初步识别灯条并过滤噪点
{
    if (target_contours.size())
        target_contours.clear();
    if (rot_rect_list.size())
        rot_rect_list.clear();
    if (rect_list.size())
        rect_list.clear();
    if (target_size_list.size())
        target_size_list.clear();

    for (int i = 0; i < contours.size(); i++)
    {
        struct TargetSize target_size;

        // 1.轮廓长度
        target_size.len = cv::arcLength(contours[i], true);
        if (target_size.len < 100 || target_size.len > 500)
            continue;

        // 2.长宽比
        cv::RotatedRect rot_rect = adjustRect(cv::minAreaRect(contours[i]));
        target_size.shortwidth = rot_rect.size.width;
        target_size.ratio = rot_rect.size.height * 1.0 / rot_rect.size.width;
        /*   if (target_size.ratio < 1 || target_size.ratio > 9)
            {

                continue;
            }*/


            //// 3.面积
        target_size.area = cv::contourArea(contours[i]);
        /* if (target_size.area < 100 || target_size.area > 2000)
         {

             continue;
         }*/


         //// 4.矩形度
        target_size.area_ratio = target_size.area * 1.0 / rot_rect.size.area();
        /*   if (target_size.area_ratio<0.6)
           {

               continue;
           }*/


           //// 5.面积比长度（似圆度）
        target_size.area_len_ratio = target_size.area * 1. / target_size.len;
        //if ()
        //{

        //    continue;
        //}


        // 6.多边形逼近
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.01 * target_size.len, true);
        target_size.corners = approx.size();
        if (target_size.corners < 5)
        {

            continue;
        }


        target_contours.push_back(contours[i]);
        rot_rect_list.push_back(rot_rect);
        rect_list.push_back(cv::boundingRect(contours[i]));
        target_size_list.push_back(target_size);
    }
}

void  ArmorDetect::filterEnemyLight(
    std::vector<std::vector<cv::Point>> light_contours,
    std::vector<cv::RotatedRect> rot_rect_list,
    std::vector<cv::Rect> rect_list,
    std::vector<struct TargetSize> target_size_list,
    std::vector<LightInfo>& light_info)//二次筛选
{
    for (int i = 0; i < light_contours.size(); i++)
    {
        LightInfo light_info_i;
        cv::RotatedRect rot_rect = adjustRect(rot_rect_list[i]);

        // 1.角度
        double angle = rot_rect.angle;
        angle = 90 - angle;
        angle = angle < 0.0 ? angle + 180 : angle;
        float delta_angle = abs(angle - 90);
        if (delta_angle > 48)
        {

            continue;
        }




        cv::Rect rect = rect_list[i];//第i个矩形

        light_info_i.idx = i;
        light_info_i.contours.assign(light_contours[i].begin(), light_contours[i].end());
        light_info_i.size = target_size_list[i];
        light_info_i.rrect = rot_rect;
        light_info_i.rect = rect_list[i];
        light_info.push_back(light_info_i);
    }
}

void ArmorDetect::findArmor(std::vector<LightInfo>& light_list, Mat& frame)//两个灯条合成一个装甲板
{
    RotatedRect rot_rect1;
    for (int i = 0; i < light_list.size(); i++)
    {
        const cv::RotatedRect& rect_i = light_list[i].rrect;
        const cv::Point& center_i = rect_i.center;
        float xi = center_i.x;
        float yi = center_i.y;
        int width = light_list[i].size.shortwidth;
        int proprotion = light_list[i].size.shortwidth;

        for (int j = i + 1; j < light_list.size(); j++)
        {
            const cv::RotatedRect& rect_j = light_list[j].rrect;
            const cv::Point& center_j = rect_j.center;
            float xj = center_j.x;
            float yj = center_j.y;
            rot_rect1 = boundingRRect(rect_i, rect_j);

            if (contourArea(light_list[i].contours) - contourArea(light_list[j].contours) > 200 ||
                contourArea(light_list[i].contours) - contourArea(light_list[j].contours) < -200)
            {
                continue;
            }



            cv::Point2f p_i[4], p_j[4];
            rect_i.points(p_i);
            rect_j.points(p_j);




            std::vector<cv::Point2f> p_list = {
                (p_i[1] + p_i[2]) / 2,
                (p_i[3] + p_i[0]) / 2,
                (p_j[1] + p_j[2]) / 2,
                (p_j[3] + p_j[0]) / 2
            };

            //装甲板矩形角点排序
            std::sort(p_list.begin(), p_list.end(), [](cv::Point2f& p1, cv::Point2f& p2)->bool {return p1.x < p2.x; });

            if (p_list[0].y > p_list[1].y)
            {
                cv::Point2f point_temp = p_list[0];
                p_list[0] = p_list[1];
                p_list[1] = point_temp;
            }
            if (p_list[2].y < p_list[3].y)
            {
                cv::Point2f point_temp = p_list[2];
                p_list[2] = p_list[3];
                p_list[3] = point_temp;
            }



            armor.rrect = rot_rect1;
            armor.pos.p.clear();
            armor.pos.p.push_back(p_list[0]);
            armor.pos.p.push_back(p_list[1]);
            armor.pos.p.push_back(p_list[2]);
            armor.pos.p.push_back(p_list[3]);
        }
    }
}

RotatedRect ArmorDetect::boundingRRect(const cv::RotatedRect& left, const cv::RotatedRect& right)//通过两个灯条绘制旋转矩形
{
    const cv::Point& pl = left.center, & pr = right.center;
    cv::Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}


RotatedRect ArmorDetect::adjustRect(const cv::RotatedRect& rect)//调整旋转矩形
{
    const cv::Size2f& s = rect.size;
    if (s.width < s.height)
        return rect;
    return cv::RotatedRect(rect.center, cv::Size2f(s.height, s.width), rect.angle + 90.0);
}
void ArmorDetect::getTarget2dPoinstion(const cv::RotatedRect& rect, std::vector<cv::Point2f>& target2d)//对装甲板进行角点排序
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.x < p2.x; });//排序算法
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);
}





void ArmorDetect::findRune(std::vector<std::vector<cv::Point>> contours, vector<ArmorInfo>& Rune)//匹配符文板
{
    for (int i = 0; i < contours.size(); i++)
    {
        ArmorInfo temp;
        RotatedRect RRect = minAreaRect(contours[i]);
        int area = RRect.size.area();//得到面积
        ;      cv::RotatedRect rot_rect = adjustRect(cv::minAreaRect(contours[i]));
        double shortwidth = rot_rect.size.width;
        double ratio = rot_rect.size.height * 1.0 / rot_rect.size.width;
        if (ratio < 1 || ratio > 3)
        {

            continue;
        }
        if (area < 2000)//过滤噪声
        {
            continue;
        }
        double len = cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.01 * len, true);
        int corners = approx.size();
        if (corners < 3 || corners>9)
        {

            continue;
        }


        vector<Point2f> object2d_point;
        getTarget2dPoinstion(RRect, object2d_point);//输入旋转矩形，返回坐标点
        temp.pos.p = object2d_point;
        temp.rrect = RRect;
        Rune.push_back(temp);








    }
}
    
    
    
    
    
    
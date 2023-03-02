#pragma once
/**
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"

using namespace cv;
using namespace std;

class Ring
{
public:
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测
    int countWide = 0; // 环岛入口变宽区域行数
    int withi1=0;
    int withi2=0;
    bool ringcircle=false; //圆环标志
    int count=0;
    int k=0;
    int finish=0;
    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        RingType ringType = RingType::RingNone; // 环岛类型
        RingStep ringStep = RingStep::None;     // 环岛处理阶段
        int rowRepairLine = 0;                  // 用于环补线的点（行号）
        int colRepairLine = 0;                  // 用于环补线的点（列号
        countWide = 0;
        withi1=0;
        withi2 =0;
        count=0;
        k=0;
        finish=0;
    }
    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool process(Tracking &track, Mat &imagePath)
    {
        // if (counterShield < 40)// 确保在环岛检测被触发后的一段时间内不会再次触发
        // {
        //     counterShield++;
        //     return false;
        // }
        bool ringEnable = false;                                 // 判环标志
        RingType ringTypeTemp = RingType::RingNone;              // 环岛类型：临时变量
        _index = 0;
        _ringPoint = POINT(0, 0);
        //std::cout<<ringStep<<std::endl;

        // 判环
        int with=0;
        bool rightpoint=false;
        //从下往上搜索
        for (int i = 1; i < track.widthBlock.size(); ++i)
        {   
            //std::cout<<"\tle:"<<track.stdevLeft<<"\tri:"<<track.stdevRight<<std::endl;
            //上行宽度大于下行，并且宽度大于图像列宽0.6并且行大于30，左点集标准差大于120，右点集标准差小于50，判断赛道平直程度
            if (track.widthBlock[i].y - track.widthBlock[i-1].y>30&&
            ((track.stdevLeft>170 &&track.stdevRight<30)||(track.stdevRight>150&&track.stdevLeft<40))&&(ringStep == RingStep::None||ringStep == RingStep::Entering)) // 搜索突然变宽的路径行数
            {
                if(track.pointsEdgeRight[i].y>track.pointsEdgeRight[i-1].y){
                    std::cout<<"进去了"<<std::endl;
                    count=0;
                    rightpoint=true;
                    withi1=i;
                    countWide++;
                    for(int ii = withi1; ii < track.widthBlock.size(); ii++){        //从找到角点下往上搜索最小点
                        if(track.pointsEdgeRight[ii+1].y<track.pointsEdgeRight[ii-10].y&&track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii+10].y){
                            ringStep = RingStep::Entering;
                            withi2=ii;
                            break;
                        }
                        
                    }
                    break;
                }
            }
            else{
                rightpoint=false;
            }
        }
        //std::cout<<rowYendStraightside<<std::endl;
                    // [1] 入环判断 countwide计数器大于五，并且存在岔路点，则认为入环
        if (ringStep == RingStep::None||ringStep  == RingStep::Entering&& withi1>1)
        {
            //std::cout<<"环岛";
            if (ringTypeTemp == RingType::RingNone) // 环岛方向判定
            {
                if (track.stdevRight<50)
                {
                    std::cout<<"左环岛";
                    ringTypeTemp = RingType::RingLeft;            // 环岛类型：左入环
                    if(ringStep  == RingStep::Entering)
                        ringType=ringTypeTemp;  
                }
                //如果存在前第5个点右集col小于当前col，说明有向右拐的趋向
                else if (track.stdevLeft<50)
                {
                    std::cout<<"右环岛";
                    ringTypeTemp = RingType::RingRight;            // 环岛类型：右入环
                    if(ringStep  == RingStep::Entering)
                        ringType= ringTypeTemp;  
                }
            }
            if(ringType==RingType::RingRight&&ringStep == RingStep::None){
                std::cout<<"进去补1线"<<std::endl;
                float k=(float)(track.pointsEdgeRight[0].y-track.pointsEdgeRight[withi1 - 5].y)/(float)(track.pointsEdgeRight[0].x-track.pointsEdgeRight[withi1 - 5].x);
                for(int n=withi1-5;n<track.pointsEdgeRight.size();n++){
                    track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi1-11].y-k*(n-withi1+10));           //进环前补线
                }
            }
            else if(ringType==RingType::RingRight&&ringStep == RingStep::Entering){
                 std::cout<<"进去补2线"<<std::endl;
                float k=(float)(track.pointsEdgeRight[0].y-track.pointsEdgeRight[withi2].y)/(float)(track.pointsEdgeRight[0].x-track.pointsEdgeRight[withi2].x);
                for(int n=withi1-2;n<withi2;n++){
                    track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi1-3].y-k*(n-withi1+2));
                }
            }
            if(ringType==RingType::RingLeft&&ringStep == RingStep::None){
                std::cout<<"进去补1线"<<std::endl;
                float k=(float)(track.pointsEdgeLeft[0].y-track.pointsEdgeLeft[withi1 - 5].y)/(float)(track.pointsEdgeLeft[0].x-track.pointsEdgeLeft[withi1 - 5].x);
                for(int n=withi1-5;n<track.pointsEdgeLeft.size();n++){
                    track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi1-11].y-k*(n-withi1+10));           //进环前补线
                }
            }
            else if(ringType==RingType::RingLeft&&ringStep == RingStep::Entering){
                 std::cout<<"进去补2线"<<std::endl;
                float k=(float)(track.pointsEdgeLeft[0].y-track.pointsEdgeLeft[withi2].y)/(float)(track.pointsEdgeLeft[0].x-track.pointsEdgeLeft[withi2].x);
                for(int n=withi1-2;n<withi2;n++){
                    track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi1-3].y-k*(n-withi1+2));
                }
            }
            //std::cout<<ringTypeTemp<<std::endl;;
        }
        if(ringStep == RingStep::Entering||ringStep == RingStep::Inside){
            for (int i = track.widthBlock.size(); i >1; i--){
                if(track.widthBlock[i].y-track.widthBlock[i+1].y>100&&track.widthBlock[i].y>COLSIMAGE*0.65&&
                track.widthBlock[i+1].y<COLSIMAGE*0.5&&track.widthBlock[i].x<150&&rightpoint==false){
                    std::cout<<"环补线找到"<<std::endl;
                    withi2=track.widthBlock[i].x;
                    ringStep = RingStep::Inside;
                    ringcircle=true;
                    break;
                }
                if( i < 3 && ringcircle==true){
                    count++;
                    if(count>32){
                        ringStep = RingStep::Circle;
                    }
                }
            }
        }
        if(ringStep == RingStep::Inside){
            if(ringType== RingType::RingRight){
                std::cout<<"布线"<<std::endl;
                for (int i = 0; i < COLSIMAGE; i++)
                {
                    circle(imagePath, Point(i,withi2), 2,
                        Scalar(0, 0, 255), -1); // 红色点
                }
                int n=track.pointsEdgeRight.size();
                int x= track.pointsEdgeRight[n-withi2].x*1.2;
                int y= track.pointsEdgeRight[n-withi2].y*0.8;
                POINT startPoint=track.pointsEdgeLeft[0];
                for(int i=COLSIMAGE;i>COLSIMAGE*0.8;i--){
                    startPoint = track.pointsEdgeLeft[COLSIMAGE-i];
                    if( track.pointsEdgeLeft[COLSIMAGE-i].y>2)
                        break;
                }
                POINT midPoint(x, y);                                            // 补线：中点
                POINT endPoint(withi2-10,track.pointsEdgeRight[withi2-10].y);                          // 补线：终点

                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeLeft.resize( b_modify.size());
                track.pointsEdgeRight.resize( b_modify.size());
                for (int kk = 1; kk < b_modify.size(); ++kk)
                {
                    track.pointsEdgeLeft[kk]=b_modify[kk];
                }
            }
            if(ringType== RingType::RingLeft){
                std::cout<<"布线"<<std::endl;
                for (int i = 0; i < COLSIMAGE; i++)
                {
                    circle(imagePath, Point(i,withi2), 2,
                        Scalar(0, 0, 255), -1); // 红色点
                }
                int n= track.pointsEdgeLeft.size();
                int x= track.pointsEdgeLeft[n-withi2].x*0.8;
                int y= track.pointsEdgeLeft[n-withi2].y*0.12;
                POINT startPoint=track.pointsEdgeRight[0];
                for(int i=COLSIMAGE;i>COLSIMAGE*0.8;i--){
                    startPoint = track.pointsEdgeRight[COLSIMAGE-i];
                    if( track.pointsEdgeRight[COLSIMAGE-i].y<COLSIMAGE)
                        break;
                }
                POINT midPoint(x, y);                                            // 补线：中点
                POINT endPoint(withi2-10,track.pointsEdgeRight[withi2-10].y);                          // 补线：终点

                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeRight.resize( b_modify.size());
                track.pointsEdgeLeft.resize( b_modify.size());
                for (int kk = 1; kk < b_modify.size(); ++kk)
                {
                    track.pointsEdgeRight[kk]=b_modify[kk];
                }
            }
        }
        if(ringStep == RingStep::Circle || ringStep == RingStep::Exiting){
            for(int i=1;i<track.widthBlock.size();i++){
                if(track.widthBlock[i].y<track.widthBlock[i-3].y&&track.widthBlock[i].y<track.widthBlock[i+3].y){
                    withi2=track.widthBlock[i].x;
                    break;
                }
            }
            if(ringType == RingType::RingRight){
                if((track.pointsEdgeLeft.size()> ROWSIMAGE*0.6&& withi2 <ROWSIMAGE*0.85&&withi2 >ROWSIMAGE*0.4)||(finish<3&&ringStep == RingStep::Exiting)){
                    k++;
                    std::cout<<"出环岛"<<withi2<<std::endl;
                    for (int i = 0; i < COLSIMAGE; i++)
                        {
                            circle(imagePath, Point(i,withi2), 2,
                                Scalar(0, 0, 255), -1); // 红色点
                        }
                    int x= track.pointsEdgeLeft[withi2+5].x;
                    int y= track.pointsEdgeLeft[withi2+5].y;
                    POINT startPoint=track.pointsEdgeLeft[0];
                    POINT midPoint(x, y);                                            // 补线：中点
                    POINT endPoint = track.pointsEdgeRight[withi2-1];
                    for(int ii=0;ii<track.pointsEdgeRight.size();ii++){
                        if(track.pointsEdgeRight[ii].y<COLSIMAGE-2&&track.pointsEdgeRight[ii].x<withi2){
                            endPoint.x=track.pointsEdgeRight[ii].x;
                            endPoint.y=640;
                            break;
                        }
                    }                        // 补线：终点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> b_modify = Bezier(0.01, input);
                    track.pointsEdgeLeft.resize( b_modify.size());
                    track.pointsEdgeRight.resize( b_modify.size());
                    for (int kk = 1; kk < b_modify.size(); ++kk)
                    {
                        track.pointsEdgeLeft[kk]=b_modify[kk];
                    }
                }
            }
            if(ringType == RingType::RingLeft){
                if(track.pointsEdgeRight.size()> ROWSIMAGE*0.6&& withi2 <ROWSIMAGE*0.85&&withi2 >ROWSIMAGE*0.4){
                    k++;
                    std::cout<<"出环岛"<<withi2<<std::endl;
                    for (int i = 0; i < COLSIMAGE; i++)
                        {
                            circle(imagePath, Point(i,withi2), 2,
                                Scalar(0, 0, 255), -1); // 红色点
                        }
                    int n= track.pointsEdgeRight.size();
                    int x= track.pointsEdgeRight[n-withi2].x*0.8;
                    int y= track.pointsEdgeRight[n-withi2].y*0.8;
                    POINT startPoint=track.pointsEdgeRight[0];
                    POINT midPoint(x, y);                                            // 补线：中点
                    POINT endPoint = track.pointsEdgeLeft[withi2-1];
                    for(int ii=0;ii<track.pointsEdgeLeft.size();ii++){
                        if(track.pointsEdgeLeft[ii].y<COLSIMAGE){
                            endPoint.x=track.pointsEdgeLeft[ii].x;
                            endPoint.y=640;
                            break;
                        }
                    }                        // 补线：终点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> b_modify = Bezier(0.01, input);
                    track.pointsEdgeLeft.resize( b_modify.size());
                    track.pointsEdgeRight.resize( b_modify.size());
                    for (int kk = 1; kk < b_modify.size(); ++kk)
                    {
                        track.pointsEdgeLeft[kk]=b_modify[kk];
                    }
                }
            }
                if(k>10){
                    for(int i=track.widthBlock.size();track.widthBlock[i].x>ROWSIMAGE*0.6;i++){
                        if(track.widthBlock[i].x-track.widthBlock[i-2].x>30)
                            ringStep = RingStep::Exiting;
                    }
                }
        }
        if(ringStep == RingStep::Exiting){
            finish++;
            if(finish>35){
                ringStep = RingStep::None;
                reset();
            }
        }
        // 返回识别结果
        if (ringStep == RingStep::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 绘制环岛识别图像
     *
     * @param ringImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &ringImage)
    {
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        putText(ringImage, to_string(_ringStep) + " " + to_string(_ringEnable) + " " + to_string(_tmp_ttttt),
                Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, to_string(track.validRowsRight) + " | " + to_string(track.stdevRight),
                Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, cv::LINE_AA);
        putText(ringImage, to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
                Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, "[7] RING - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0), -1); // 蓝色点，入环点
    }

private:
    uint16_t counterSpurroad = 0; // 岔路计数器
    // 临时测试用参数
    int _ringStep;
    int _ringEnable;
    int _tmp_ttttt;
    int _index = 0;
    POINT _ringPoint = POINT(0, 0);

    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Circle,
        Exiting,  // 出环
        Finish    // 环任务结束
    };
        RingStep ringStep = RingStep::None;     // 环岛处理阶段
    RingType ringType = RingType::RingNone; // 环岛类型
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
};
#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"

using namespace cv;
using namespace std;

class Tracking
{
public:
    vector<POINT> pointsEdgeLeft;     // 赛道左边缘点集
    vector<POINT> pointsEdgeRight;    // 赛道右边缘点集
    vector<POINT> widthBlock;         // 色块宽度=终-起（每行）存储赛道宽度信息，每个元素表示一行中赛道的起始和终止点。x->row y->width
    vector<POINT> spurroad;           // 保存岔路信息
    double stdevLeft;                 // 边缘斜率方差（左）
    double stdevRight;                // 边缘斜率方差（右）评估赛道线的直线性
    int validRowsLeft = 0;            // 边缘有效行数（左）
    int validRowsRight = 0;           // 边缘有效行数（右）
    POINT garageEnable = POINT(0, 0); // 车库识别标志：（x=1/0，y=row)x=1表示已识别，y存储识别行。
    uint16_t rowCutUp = 10;           // 图像顶部切行
    uint16_t rowCutBottom = 10;       // 图像底部切行 图像顶部和底部切行的数量，用于裁剪图像以减少处理区域。


    /**
     * @brief 赛道线识别
     *
     * @param isResearch 是否重复搜索
     * @param rowStart 边缘搜索起始行
     */
    void trackRecognition(bool isResearch, uint16_t rowStart)
    {
        bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
        int counterSearchRows = pointsEdgeLeft.size(); // 搜索行计数
        int startBlock[30];                            // 色块起点（行）
        int endBlock[30];                              // 色块终点（行）
        int counterBlock = 0;                          // 色块计数器（行）
        POINT pointSpurroad;                           // 岔路坐标
        int counterSpurroad = 0;                       // 岔路识别标志
        bool spurroadEnable = false;                   //用来指示是否启用岔路识别。在这里被初始化为false，意味着在开始搜索时，默认不识别岔路。

        if (rowCutUp > ROWSIMAGE / 2)                 //为了防止切除过多的行数，这里将它们的最大值限制在图像总行数的四分之一
            rowCutUp = ROWSIMAGE / 2;
        if (rowCutBottom > ROWSIMAGE / 2)
            rowCutBottom = ROWSIMAGE / 2;

        if (!isResearch)
        {
            pointsEdgeLeft.clear();              // 初始化边缘结果
            pointsEdgeRight.clear();             // 初始化边缘结果
            widthBlock.clear();                  // 初始化色块数据
            spurroad.clear();                    // 岔路信息
            validRowsLeft = 0;                   // 边缘有效行数（左）
            validRowsRight = 0;                  // 边缘有效行数（右）
            flagStartBlock = true;               // 搜索到色块起始行的标志（行）
            garageEnable = POINT(0, 0);          // 车库识别标志初始化
            rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
        }
        else
        {
            if (pointsEdgeLeft.size() > rowStart)//赛道线识别的重复搜索模式下，调整各个存储边缘点和宽度信息的数据结构的大小,
                                                 // 并更新搜索的起始点和状态，以便从特定的行开始重新识别赛道线。
                pointsEdgeLeft.resize(rowStart);
            if (pointsEdgeRight.size() > rowStart)
                pointsEdgeRight.resize(rowStart);
            if (widthBlock.size() > rowStart)
            {
                widthBlock.resize(rowStart);
                if (rowStart > 1)
                    rowStart = widthBlock[rowStart - 1].x - 2;
            }

            flagStartBlock = false; // 搜索到色块起始行的标志（行）
        }

        //  开始识别赛道左右边缘
        for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
        {
            counterBlock = 0; // 色块计数器清空
            // 搜索色（block）块信息

            //RGB图像
            if (imageType == ImageType::Rgb) // 输入RGB图像
            {
                if (imagePath.at<Vec3b>(row, 1)[2] > 0)
                {
                    startBlock[counterBlock] = 0;
                }
                for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
                {
                    if (imagePath.at<Vec3b>(row, col)[2] > 0 &&
                        imagePath.at<Vec3b>(row, col - 1)[2] == 0)
                    {
                        startBlock[counterBlock] = col;
                    }
                    else
                    {
                        if (imagePath.at<Vec3b>(row, col)[2] == 0 &&
                            imagePath.at<Vec3b>(row, col - 1)[2] > 0)
                        {
                            endBlock[counterBlock++] = col;
                            if (counterBlock >= end(endBlock) - begin(endBlock))
                                break;
                        }
                    }
                }
                if (imagePath.at<Vec3b>(row, COLSIMAGE - 1)[2] > 0)
                {
                    if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                        endBlock[counterBlock++] = COLSIMAGE - 1;
                }
            }

            // 二值化图像
            if (imageType == ImageType::Binary) // 输入二值化图像
            {
                if (imagePath.at<uchar>(row, 1) > 127)//第一列如果是白色
                {
                    startBlock[counterBlock] = 0;//色块初始值为0
                }
                for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
                {
                    if (imagePath.at<uchar>(row, col) > 127 &&
                        imagePath.at<uchar>(row, col - 1) <= 127)//寻找到黑->白的变化色块
                    {
                        startBlock[counterBlock] = col;//记录色块起始位置
                    }
                    else
                    {
                        if (imagePath.at<uchar>(row, col) <= 127 &&
                            imagePath.at<uchar>(row, col - 1) > 127)//寻找到白->黑的变化色块
                        {
                            endBlock[counterBlock++] = col;//记录色块结束位置
                            if (counterBlock >= end(endBlock) - begin(endBlock))//判断是否超出数组
                                break;
                        }
                    }
                }
                if (imagePath.at<uchar>(row, COLSIMAGE - 1) > 127)//最后一列如果是白色
                {
                    if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                        endBlock[counterBlock++] = COLSIMAGE - 1;
                }
            }

            int widthBlocks = endBlock[0] - startBlock[0]; // 色块宽度临时变量
            int indexWidestBlock = 0;                      // 最宽色块的序号
            if (flagStartBlock)                            // 起始行做特殊处理(第一行rowstart)
            {
                if (row < ROWSIMAGE / 3)// 首行不满足宽度要求(第一行那有这么小的)
                    return;
                if (counterBlock == 0)// 无色块
                {
                    continue;
                }
                for (int i = 1; i < counterBlock; i++) // 搜索最宽色块
                {
                    int tmp_width = endBlock[i] - startBlock[i];
                    if (tmp_width > widthBlocks)
                    {
                        widthBlocks = tmp_width;
                        indexWidestBlock = i;
                    }
                }

                int limitWidthBlock = COLSIMAGE * 0.3; // 首行色块宽度限制（不能太小）0.8的图像大小
                if (row < ROWSIMAGE * 0.6)
                {
                    limitWidthBlock = COLSIMAGE * 0.4;
                }
                if (widthBlocks > limitWidthBlock) // 满足首行宽度要求
                {
                    flagStartBlock = false;//第一行处理完成
                    POINT pointTmp(row, startBlock[indexWidestBlock]);//记录最大的色块起始点x,y
                    pointsEdgeLeft.push_back(pointTmp);//记录至左点集
                    pointTmp.y = endBlock[indexWidestBlock];//记录最大的色块结束点x,y
                    pointsEdgeRight.push_back(pointTmp);//记录至右点集
                    widthBlock.emplace_back(row, endBlock[indexWidestBlock] - startBlock[indexWidestBlock]);//记录色块宽度row, width
                    counterSearchRows++;
                }
                spurroadEnable = false;//起始行不搜索岔路
            }
            else // 其它行色块坐标处理
            {
                if (counterBlock == 0)
                {
                    break;
                }

                //-------------------------------------------------<车库标识识别>-------------------------------------------------------------
                if (counterBlock > 5 && !garageEnable.x)//色块数量大于五而且未标识
                {
                    int widthThis = 0;        // 色块的宽度
                    int widthVer = 0;         // 当前行色块的平均值
                    vector<int> widthGarage;  // 当前行色块宽度集合
                    vector<int> centerGarage; // 当前行色块质心集合
                    vector<int> indexGarage;  // 当前行有效色块的序号

                    for (int i = 0; i < counterBlock; i++)
                    {
                        widthThis = endBlock[i] - startBlock[i];        // 色块的宽度
                        int center = (endBlock[i] + startBlock[i]) / 2; // 色块的质心
                        if (widthThis > 5 && widthThis < 50)            // 过滤无效色块区域：噪点
                        {
                            centerGarage.push_back(center);
                            widthGarage.push_back(widthThis);
                        }
                    }

                    int widthMiddle = getMiddleValue(widthGarage); // 斑马线色块宽度中值

                    for (int i = 0; i < widthGarage.size(); i++)
                    {
                        if (abs(widthGarage[i] - widthMiddle) < widthMiddle / 3)// 验证斑马线色块宽度是否符合
                        {
                            indexGarage.push_back(i);
                        }
                    }
                    if (indexGarage.size() >= 4) // 验证有效斑马线色块个数
                    {
                        vector<int> distance;
                        for (int i = 1; i < indexGarage.size(); i++) // 质心间距的方差校验(?这里有用到质心吗?)
                        {
                            distance.push_back(widthGarage[indexGarage[i]] - widthGarage[indexGarage[i - 1]]);
                        }
                        double var = sigma(distance);//计算间距方差
                        if (var < 5.0) // 经验参数
                        {
                            garageEnable.x = 1;                      // 车库标志使能
                            garageEnable.y = pointsEdgeRight.size(); // 斑马线行序号
                        }
                    }
                }
                //------------------------------------------------------------------------------------------------------------------------

                vector<int> indexBlocks;               // 色块序号（行）
                for (int i = 0; i < counterBlock; i++) // 上下行色块的连通性判断
                {
                    //判断是否连通
                    int g_cover = min(endBlock[i], pointsEdgeRight[pointsEdgeRight.size() - 1].y) -
                                  max(startBlock[i], pointsEdgeLeft[pointsEdgeLeft.size() - 1].y);
                    if (g_cover >= 0)
                    {
                        indexBlocks.push_back(i);
                    }
                }

                if (indexBlocks.size() == 0) // 如果没有发现联通色块，则图像搜索完成，结束任务
                {
                    break;
                }
                else if (indexBlocks.size() == 1) // 只存在单个色块，正常情况，提取边缘信息
                {
                    if (endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]] < COLSIMAGE / 10)//色块宽度过小
                    {
                        continue;
                    }
                    pointsEdgeLeft.emplace_back(row, startBlock[indexBlocks[0]]);
                    pointsEdgeRight.emplace_back(row, endBlock[indexBlocks[0]]);
                    slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1); // 边缘斜率计算
                    slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                    widthBlock.emplace_back(row, endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]]);
                    spurroadEnable = false;
                }
                else if (indexBlocks.size() > 1) // 存在多个色块，则需要择优处理：选取与上一行最近的色块
                {
                    int centerLast = COLSIMAGE / 2;
                    if (pointsEdgeRight.size() > 0 && pointsEdgeLeft.size() > 0)
                        centerLast = (pointsEdgeRight[pointsEdgeRight.size() - 1].y + pointsEdgeLeft[pointsEdgeLeft.size() - 1].y) / 2; // 上一行色块的中心点横坐标
                    int centerThis = (startBlock[indexBlocks[0]] + endBlock[indexBlocks[0]]) / 2;                                       // 当前行色块的中心点横坐标
                    int differBlocks = abs(centerThis - centerLast);                                                                    // 上下行色块的中心距离
                    int indexGoalBlock = 0;                                                                                             // 目标色块的编号
                    int startBlockNear = startBlock[indexBlocks[0]];                                                                    // 搜索与上一行最近的色块起点
                    int endBlockNear = endBlock[indexBlocks[0]];                                                                        // 搜索与上一行最近的色块终点

                    for (uint16_t i = 1; i < indexBlocks.size(); i++) // 搜索与上一行最近的色块编号
                    {
                        centerThis = (startBlock[indexBlocks[i]] + endBlock[indexBlocks[i]]) / 2;
                        if (abs(centerThis - centerLast) < differBlocks)
                        {
                            differBlocks = abs(centerThis - centerLast);
                            indexGoalBlock = i;
                        }
                        // 搜索与上一行最近的边缘起点和终点
                        if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[i]]) <
                            abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                        {
                            startBlockNear = startBlock[indexBlocks[i]];
                        }
                        if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[i]]) <
                            abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                        {
                            endBlockNear = endBlock[indexBlocks[i]];
                        }
                    }

                    // 检索最佳的起点与终点
                    if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[indexGoalBlock]]) <
                        abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                    {
                        startBlockNear = startBlock[indexBlocks[indexGoalBlock]];
                    }
                    if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[indexGoalBlock]]) <
                        abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                    {
                        endBlockNear = endBlock[indexBlocks[indexGoalBlock]];
                    }

                    if (endBlockNear - startBlockNear < COLSIMAGE / 10)
                    {
                        continue;
                    }
                    POINT tmp_point(row, startBlockNear);
                    pointsEdgeLeft.push_back(tmp_point);
                    tmp_point.y = endBlockNear;
                    pointsEdgeRight.push_back(tmp_point);
                    widthBlock.emplace_back(row, endBlockNear - startBlockNear);
                    slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1);
                    slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                    counterSearchRows++;

                    //-------------------------------<岔路信息提取>----------------------------------------
                    pointSpurroad.x = row;
                    pointSpurroad.y = endBlock[indexBlocks[0]];
                    if (!spurroadEnable)
                    {
                        spurroad.push_back(pointSpurroad);
                        spurroadEnable = true;
                    }
                    //------------------------------------------------------------------------------------
                }

                stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
                stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

                validRowsCal(); // 有效行计算
            }
        }
    }

    /**
     * @brief 赛道线识别
     *
     * @param imageBinary 赛道识别基准图像
     */
    void trackRecognition(Mat &imageBinary)
    {
        imagePath = imageBinary;
        trackRecognition(false, 0);
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param trackImage 需要叠加显示的图像
     */
    void drawImage(Mat &trackImage)
    {
        for (uint16_t i = 0; i < pointsEdgeLeft.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (uint16_t i = 0; i < pointsEdgeRight.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        for (uint16_t i = 0; i < spurroad.size(); i++)
        {
            circle(trackImage, Point(spurroad[i].y, spurroad[i].x), 3,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_8U);
        putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_8U);
    }

    /**
     * @brief 边缘斜率计算，计算边缘点集的标准差，用来衡量边缘的稳定性或波动程度。
     *
     * @param v_edge
     * @param img_height
     * @return double
     */
    double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    {
        if (v_edge.size() < img_height / 4)
        {
            return 1000;
        }
        vector<int> v_slope;
        int step = 10; // v_edge.size()/10;
        for (uint16_t i = step; i < v_edge.size(); i += step)
        {
            if (v_edge[i].x - v_edge[i - step].x)
                v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 / (v_edge[i].x - v_edge[i - step].x));
        }
        if (v_slope.size() > 1)
        {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope), [&](const double d)
                     { accum += (d - mean) * (d - mean); });

            return sqrt(accum / (v_slope.size() - 1)); // 方差
        }
        else
            return 0;
    }

private:
    Mat imagePath; // 赛道搜索图像
    /**
     * @brief 赛道识别输入图像类型
     *
     */
    enum ImageType
    {
        Binary = 0, // 二值化
        Rgb,        // RGB
    };

    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：二值化图像
    /**
     * @brief 边缘斜率计算
     *
     * @param edge
     * @param index
     */
    void slopeCal(vector<POINT> &edge, int index)
    {
        if (index <= 4)
        {
            return;
        }
        float temp_slop1 = 0.0, temp_slop2 = 0.0;
        if (edge[index].x - edge[index - 2].x != 0)
        {
            temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                         ((edge[index].x - edge[index - 2].x) * 1.0f);
        }
        else
        {
            temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
        }
        if (edge[index].x - edge[index - 4].x != 0)
        {
            temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                         ((edge[index].x - edge[index - 4].x) * 1.0f);
        }
        else
        {
            edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
        }
        if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
        {
            edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
        }
        else if (abs(temp_slop1) != 255)
        {
            edge[index].slope = temp_slop1;
        }
        else
        {
            edge[index].slope = temp_slop2;
        }
    }

    /**
     * @brief 边缘有效行计算：左/右
     *
     */
    void validRowsCal(void)
    {
        // 左边有效行
        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsEdgeRight.size() > 1)
        {
            for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }

    /**
     * @brief 冒泡法求取集合中值
     *
     * @param vec 输入集合
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec)
    {
        if (vec.size() < 1)
            return -1;
        if (vec.size() == 1)
            return vec[0];

        int len = vec.size();
        while (len > 0)
        {
            bool sort = true; // 是否进行排序操作标志
            for (int i = 0; i < len - 1; ++i)
            {
                if (vec[i] > vec[i + 1])
                {
                    swap(vec[i], vec[i + 1]);
                    sort = false;
                }
            }
            if (sort) // 排序完成
                break;

            --len;
        }

        return vec[(int)vec.size() / 2];
    }
};

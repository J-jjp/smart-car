/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file racing.cpp
 * @author Leo
 * @brief 追逐区检测
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

class Racing
{
public:
    bool carStoping = false; // 停车标志

    /**
     * @brief 检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool process(Tracking &track, vector<PredictResult> predicts)
    {
        _index = 0;
        carStoping = false; // 停车标志
        switch (typeRace)
        {
        case TypeRace::None:          // AI检测
            searchTypeRace(predicts); // 检索AI场景类型
            break;

        case TypeRace::Safe: // 普通车辆
        {
            PredictResult predict = searchSign(predicts, LABEL_SAFETY);
            counterSes[0]++;
            if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
            {
                // 检索小车靠近左/右方向
                sideLeft = false; // 靠左侧标志
                int row = track.pointsEdgeLeft[0].x - predict.y - predict.height;
                if (row < 0)
                {
                    _index = 1;
                    if (abs(track.pointsEdgeLeft[0].y - predict.x) > abs(track.pointsEdgeRight[0].y - predict.x - predict.width))
                        sideLeft = true;
                }
                else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                {
                    _index = 2;
                    if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y - predict.x - predict.width))
                        sideLeft = true;
                }
                else
                {
                    _index = 3;
                    if (abs(track.pointsEdgeLeft[row].y - predict.x) > abs(track.pointsEdgeRight[row].y - predict.x - predict.width))
                        sideLeft = true;
                }

                curtailTracking(track, sideLeft); // 缩减优化车道线（双车道→单车道）
                counterSes[0] = 0;
            }
            else if (counterSes[0] > 10) // AI检测异常：退出该场景
            {
                counterRec[0] = 0;
                counterSes[0] = 0;
                typeRace = TypeRace::None;
            }
            break;
        }

        case TypeRace::Spy: // 嫌疑车辆
        {
            /**
             *嫌疑车辆逼停策略：绕行至车辆前方阻挡其运行
             */
            if (stepSpy == StepSpy::Det) // AI检测嫌疑车辆所在赛道两侧
            {
                PredictResult predict = searchSign(predicts, LABEL_SPY);
                counterSes[1]++;
                if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
                {
                    // 检索小车靠近左/右方向
                    sideLeft = true; // 靠左侧标志
                    int row = track.pointsEdgeLeft[0].x - predict.y - predict.height;
                    if (row < 0)
                    {
                        _index = 1;
                        if (abs(track.pointsEdgeLeft[0].y - predict.x) > abs(track.pointsEdgeRight[0].y - predict.x - predict.width))
                            sideLeft = false;
                    }
                    else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                    {
                        _index = 2;
                        if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y - predict.x - predict.width))
                            sideLeft = false;
                    }
                    else
                    {
                        _index = 3;
                        if (abs(track.pointsEdgeLeft[row].y - predict.x) > abs(track.pointsEdgeRight[row].y - predict.x - predict.width))
                            sideLeft = false;
                    }

                    counterSes[1] = 0;
                    counterRec[1]++;
                    if (counterRec[1] > 5)
                    {
                        counterRec[1] = 0;
                        counterSes[1] = 0;
                        stepSpy = StepSpy::Bypass;
                    }
                }
                else if (counterSes[1] > 20) // AI检测异常：退出该场景
                {
                    counterRec[1] = 0;
                    counterSes[1] = 0;
                    typeRace = TypeRace::None;
                }
            }
            else if (stepSpy == StepSpy::Bypass) // 车辆绕行阶段
            {
                PredictResult predict = searchSign(predicts, LABEL_SPY);
                counterSes[1]++;
                if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
                    counterSes[1] = 0;
                else if (counterSes[1] > 5) // 绕行完毕
                {
                    counterSes[1] = 0;
                    stepSpy = StepSpy::Inside;
                }
            }
            else if (stepSpy == StepSpy::Inside) // 车辆变道
            {
                curtailTracking(track, sideLeft); // 缩减优化车道线（双车道→单车道）
                counterSes[1]++;
                if (counterSes[1] > 10) // 变道完毕
                {
                    counterSes[1] = 0;
                    stepSpy = StepSpy::Resist;
                }
            }
            else if (stepSpy == StepSpy::Resist) // 停车阻挡逼停
            {
                carStoping = true;
                counterSes[1]++;
                if (counterSes[1] > 50) // 停车逼停时间: 2.3s
                {
                    carStoping = false;
                    counterSes[1] = 0;
                    typeRace = TypeRace::None; // 完成，退出场景
                }
            }

            break;
        }

        case TypeRace::Danger: // 危险车辆
        {
            /**
             *恐怖车辆逼停策略：沿赛道左/右侧通行，强行撞击车辆逼停
             */
            PredictResult predict = searchSign(predicts, LABEL_DANGER);
            counterSes[2]++;
            if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
            {
                // 检索小车靠近左/右方向
                sideLeft = true; // 靠左侧标志
                int row = track.pointsEdgeLeft[0].x - predict.y - predict.height;
                if (row < 0)
                {
                    if (abs(track.pointsEdgeLeft[0].y - predict.x) > abs(track.pointsEdgeRight[0].y - predict.x - predict.width))
                        sideLeft = false;
                }
                else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                {
                    if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y - predict.x - predict.width))
                        sideLeft = false;
                }
                else
                {
                    if (abs(track.pointsEdgeLeft[row].y - predict.x) > abs(track.pointsEdgeRight[row].y - predict.x - predict.width))
                        sideLeft = false;
                }

                curtailTracking(track, sideLeft); // 缩减优化车道线（双车道→单车道）
                counterSes[2] = 0;
            }
            else if (counterSes[2] > 10) // 退出该场景
            {
                counterSes[2] = 0;
                typeRace = TypeRace::None;
            }
            break;
        }
        }

        if (typeRace == TypeRace::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (typeRace == TypeRace::Spy)
        {
            switch (stepSpy)
            {
            case StepSpy::Det:
                putText(img, "[4] RACE - SPY - Det", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Bypass:
                putText(img, "[4] RACE - SPY - Bypass", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Inside:
                putText(img, "[4] RACE - SPY - Inside", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Resist:
                putText(img, "[4] RACE - SPY - Resist", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            default:
                break;
            }
        }
        else if (typeRace == TypeRace::Danger)
            putText(img, "[4] RACE - DANGER", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        else if (typeRace == TypeRace::Safe)
            putText(img, "[4] RACE - Safe", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        putText(img, to_string(_index), Point(COLSIMAGE / 2 - 10, 40), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    int counterSes[3] = {0, 0, 0}; // 图像场次计数器
    int counterRec[3] = {0, 0, 0}; // 标志检测计数器
    bool sideLeft = true;          // AI标识左右侧使能
    int _index = 0;
    /**
     * @brief 场景类型
     *
     */
    enum TypeRace
    {
        None = 0, // AI检测
        Safe,     // 普通车辆
        Spy,      // 嫌疑车辆
        Danger    // 危险车辆
    };
    TypeRace typeRace = TypeRace::None; // 场景类型

    /**
     * @brief 嫌疑车辆逼停阶段
     *
     */
    enum StepSpy
    {
        Det = 0, // AI检测
        Bypass,  // 车辆绕行
        Inside,  // 变道
        Resist   // 阻挡
    };

    StepSpy stepSpy = StepSpy::Det; // 嫌疑车辆逼停阶段

    /**
     * @brief 检索AI场景类型
     *
     */
    void searchTypeRace(vector<PredictResult> predicts)
    {
        // 普通车辆AI连续帧检测
        for (uint16_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_SAFETY) // 普通车辆
            {
                counterRec[0]++;
                break;
            }
        }
        // 嫌疑车辆AI连续帧检测
        for (uint16_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_SPY) // 嫌疑车辆
            {
                counterRec[1]++;
                break;
            }
        }
        // 危险车辆AI连续帧检测
        for (uint16_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_DANGER) // 危险车辆
            {
                counterRec[2]++;
                break;
            }
        }

        if (counterRec[0]) // 嫌疑车辆场景检测
        {
            counterSes[0]++;
            if (counterRec[0] > 3 && counterSes[0] <= 8)
            {
                typeRace = TypeRace::Safe; // 场景类型
                counterRec[0] = 0;
                counterSes[0] = 0;
            }
            else if (counterSes[0] > 8)
            {
                counterRec[0] = 0;
                counterSes[0] = 0;
            }
        }
        if (counterRec[1]) // 嫌疑车辆场景检测
        {
            counterSes[1]++;
            if (counterRec[1] > 3 && counterSes[1] <= 8)
            {
                typeRace = TypeRace::Spy; // 场景类型
                counterRec[1] = 0;
                counterSes[1] = 0;
                stepSpy = StepSpy::Det;
            }
            else if (counterSes[1] > 8)
            {
                counterRec[1] = 0;
                counterSes[1] = 0;
            }
        }
        if (counterRec[2]) // 危险车辆场景检测
        {
            counterSes[2]++;
            if (counterRec[2] > 3 && counterSes[2] <= 8)
            {
                typeRace = TypeRace::Danger; // 场景类型
                counterRec[2] = 0;
                counterSes[2] = 0;
            }
            else if (counterSes[2] > 8)
            {
                counterRec[2] = 0;
                counterSes[2] = 0;
            }
        }
    }

    /**
     * @brief 检索目标图像坐标
     *
     * @param predicts AI识别结果
     * @param index 检索序号
     * @return PredictResult
     */
    PredictResult searchSign(vector<PredictResult> predicts, int index)
    {
        PredictResult predict;
        predict.x = 0;
        predict.y = 0;
        predict.height = 0;
        predict.width = 0;
        // AI连续帧检测
        for (uint16_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == index)
            {
                // 通过框大小过滤最佳目标
                if (predicts[i].height * predicts[i].width > predict.height * predict.width)
                {
                    predict = predicts[i];
                }
            }
        }
        return predict;
    }

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (int i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
    }
};

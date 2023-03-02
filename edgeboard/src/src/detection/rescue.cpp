/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file rescue.cpp
 * @author Leo
 * @brief 救援区检测
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../mapping.cpp"
#include "../recognition/tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Rescue {
public:
  bool carStoping = false;  // 停车标志
  bool carExitting = false; // 出库标志

  enum Step {
    None = 0, // AI检测
    Enable,   // 使能（标志识别成功）
    Enter,    // 进站
    Cruise,   // 巡航
    Stop,     // 停车
    Exit      // 出站
  };

  Step step = Step::None;
  bool entryLeft = true; // 左入库使能标志
  /**
   * @brief 检测初始化
   *
   */
  void reset(void) {
    carStoping = false;
    carExitting = false;
    step = Step::None;
    counterSession = 0;         // 图像场次计数器
    counterRec = 0;             // 标志检测计数器
    lastPointsEdgeLeft.clear(); // 记录上一场边缘点集（丢失边）
    lastPointsEdgeRight.clear();
    counterExit = 0;
    counterImmunity = 0;
  }

  /**
   * @brief 检测与路径规划
   *
   * @param track 赛道识别结果
   * @param detection AI检测结果
   */
  bool process(Tracking &track, vector<PredictResult> predict) {
    std::cout<<step<<std::endl;
    _pointNearCone = POINT(0, 0);
    _distance = 0;
    pointConeLeft.clear();
    pointConeRight.clear();
    levelCones = 0;
    indexDebug = 0;

    switch (step) {
    case Step::None: //[01] 标志检测
      if ((counterImmunity > 200 && again) ||
          (counterImmunity > 30 && !again)) {
        for (uint16_t i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_TUMBLE ||
              predict[i].type == LABEL_PATIENT) // 伤员平民标志检测
          {
            counterRec++;
            break;
          }
        }
        for (uint16_t i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_EVIL ||
              predict[i].type == LABEL_THIEF) // 伤员平民标志检测
          {
            counterExit++;
            break;
          }
        }

        if (counterRec || counterExit) {
          counterSession++;
          if (counterRec > 3 && counterSession <= 8) {
            step = Step::Enable; // 使能
            entryLeft = true;
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
          } else if (counterExit > 3 && counterSession <= 8) {
            step = Step::Enable; // 使能
            entryLeft = false;
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
          } else if (counterSession > 8) {
            counterRec = 0;
            counterSession = 0;
          }
        }
      } else
        counterImmunity++;
      break;

    case Step::Enable: //[02] 使能
    {
      counterExit++;
      if (counterExit > 300) // 超时退出
      {
        reset();
        return false;
      }

      searchCones(predict); // 搜索赛道左右两边锥桶
      if (entryLeft)        // 左入库
      {
        _pointNearCone = getConeLeftDown(track.pointsEdgeLeft,
                                         pointConeLeft); // 搜索右下锥桶
        if (_pointNearCone.x >
            ROWSIMAGE * 0.3) // 当车辆开始靠近右边锥桶：准备入库
        {
          counterRec++;
          if (counterRec >= 2) {
            step = Step::Enter; // 进站使能
            counterRec = 0;
            counterSession = 0;
            counterExit = 0;
          }
        }
      } else // 右入库
      {
        _pointNearCone = getConeRightDown(track.pointsEdgeRight,
                                          pointConeRight); // 搜索左下锥桶
        if (_pointNearCone.x >
            ROWSIMAGE * 0.4) // 当车辆开始靠近右边锥桶：准备入库
        {
          counterRec++;
          if (counterRec >= 2) {
            step = Step::Enter; // 进站使能
            counterRec = 0;
            counterSession = 0;
            counterExit = 0;
            pathsEdgeLeft.clear();
            pathsEdgeRight.clear();
          }
        }
      }
      break;
    }
    case Step::Enter: //[03] 入库使能
    {
      counterSession++; // 屏蔽期:防止提前入库
      if (counterSession > 8) {
        if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 &&
            track.pointsEdgeRight.size() > ROWSIMAGE / 2) {
          counterExit++;
          if (counterExit > 75) {
            counterExit = 0;
            step = Step::Stop; // 停车使能
            counterRec = 0;
            counterSession = 0;
          }
        }

        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 &&
            track.pointsEdgeRight.size() < ROWSIMAGE / 2) // 赛道还未丢失
        {
          counterRec++;
          if (counterRec > 10) {
            counterRec = 0;
            step = Step::Cruise; // 巡航使能
            counterSession = 0;
          }
        }

        if (entryLeft) // 左入库
        {
          POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
          POINT end = POINT(50, 0);
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
          vector<POINT> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.05, input); // 补线
          track.pointsEdgeLeft =
              predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
          lastPointsEdgeLeft = track.pointsEdgeLeft;
          lastPointsEdgeRight = track.pointsEdgeRight;

          pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
          pathsEdgeRight.push_back(track.pointsEdgeRight);
        } else // 右入库
        {
          POINT start = POINT(ROWSIMAGE - 40, 0);
          POINT end = POINT(50, COLSIMAGE - 1);
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
          vector<POINT> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.05, input); // 补线
          track.pointsEdgeRight =
              predictEdgeRight(track.pointsEdgeLeft); // 由右边缘补偿左边缘
          lastPointsEdgeLeft = track.pointsEdgeLeft;
          lastPointsEdgeRight = track.pointsEdgeRight;

          pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
          pathsEdgeRight.push_back(track.pointsEdgeRight);
        }
      }
      break;
    }

    case Step::Cruise: //[04] 巡航使能
    {
      counterSession++;
      if (counterSession > 10) // 超时保护
      {
        step = Step::Stop; // 停车使能
        counterRec = 0;
        counterSession = 0;
        counterExit = 0;
      }

      if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 &&
          track.pointsEdgeRight.size() > ROWSIMAGE / 2) {
        counterExit++;
        if (counterExit > 15) {
          counterExit = 0;
          step = Step::Stop; // 停车使能
          counterRec = 0;
        }
      }

      searchCones(predict);             // 计算锥桶的坐标
      POINT heighestCone = POINT(0, 0); // 搜索顶点锥桶
      for (uint16_t i = 0; i < pointConeLeft.size(); i++) {
        heighestCone = pointConeLeft[0];
        if (pointConeLeft[i].x < heighestCone.x)
          heighestCone = pointConeLeft[i];
      }
      for (uint16_t i = 0; i < pointConeRight.size(); i++) {
        if (pointConeRight[i].x < heighestCone.x)
          heighestCone = pointConeRight[i];
      }

      if (heighestCone.x > 0 && heighestCone.y > 0) {
        if (heighestCone.y >= COLSIMAGE / 3) // 顶角锥桶靠近右边→继续右转
        {
          vector<POINT> points;
          for (uint16_t i = 0; i < pointConeLeft.size();
               i++) // 搜索以最高点为分界线左边的锥桶
          {
            if (pointConeLeft[i].y <= heighestCone.y)
              points.push_back(pointConeLeft[i]);
          }

          if (points.size() >= 3) // 曲线补偿
          {
            pointsSortForY(points); // 排序
            vector<POINT> input = {points[0], points[points.size() / 2],
                                   points[points.size() - 1]};
            track.pointsEdgeLeft = Bezier(0.05, input); // 补线
            track.pointsEdgeRight =
                predictEdgeRight(points); // 由左边缘补偿右边缘

            indexDebug = 1;
          } else if (points.size() >= 2) {
            pointsSortForY(points); // 排序
            POINT middle =
                POINT((points[0].x + points[points.size() - 1].x) / 2,
                      (points[0].y + points[points.size() - 1].y) / 2);
            vector<POINT> input = {points[0], middle,
                                   points[points.size() - 1]};
            track.pointsEdgeLeft = Bezier(0.05, input); // 补线
            track.pointsEdgeRight =
                predictEdgeRight(points); // 由左边缘补偿右边缘
            indexDebug = 2;
          } else if (points.size() >= 1) {
            if (points[0].x > ROWSIMAGE / 2)
              step = Step::Exit; // 出站使能
          }
        } else // 顶角锥桶靠近左边→继续右转
        {
          indexDebug = 3;
          track.pointsEdgeLeft = lastPointsEdgeLeft;
          track.pointsEdgeRight = lastPointsEdgeRight;
        }

        // 计算所有锥桶的平均高度
        int num = 0;
        for (uint16_t i = 0; i < pointConeLeft.size(); i++)
          num += pointConeLeft[i].x;
        for (uint16_t i = 0; i < pointConeRight.size(); i++)
          num += pointConeRight[i].x;

        if (pointConeLeft.size() + pointConeRight.size() > 0)
          levelCones = num / (pointConeLeft.size() + pointConeRight.size());
        else
          levelCones = 0;
        if (levelCones > ROWSIMAGE * 0.5 || levelCones == 0) {
          counterRec++;
          if (counterRec > 2) {
            step = Step::Stop; // 停车使能
            counterRec = 0;
          }
        }
      } else {
        indexDebug = 4;
        track.pointsEdgeLeft = lastPointsEdgeLeft;
        track.pointsEdgeRight = lastPointsEdgeRight;
      }

      lastPointsEdgeLeft = track.pointsEdgeLeft;
      lastPointsEdgeRight = track.pointsEdgeRight;

      pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
      pathsEdgeRight.push_back(track.pointsEdgeRight);
      break;
    }

    case Step::Stop: //[05] 停车使能
    {
      carStoping = true;
      counterRec++;
      if (counterRec > 40) // 停车：20场 = 2s
      {
        carStoping = false;
        carExitting = true;
        step = Step::Exit; // 出站使能
        counterRec = 0;
      }
      break;
    }

    case Step::Exit: //[06] 出站使能
    {
      carExitting = true;
      if (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1) {
        step = Step::None; // 出站完成
        carExitting = false;
        again = true; // 第二次进入救援区标志
        reset();
      } else {
        track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
        track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
        pathsEdgeLeft.pop_back();
        pathsEdgeRight.pop_back();
      }
      break;
    }
    }

    if (step == Step::None) // 返回控制模式标志
      return false;
    else
      return true;
  }

  /**
   * @brief 识别结果图像绘制
   *
   */
  void drawImage(Tracking track, Mat &image) {
    // 赛道边缘
    for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
             1, Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    // 入库状态
    string state = "None";
    switch (step) {
    case Step::Enable:
      state = "Enable";
      break;
    case Step::Enter:
      state = "Enter";
      break;
    case Step::Cruise:
      state = "Cruise";
      break;
    case Step::Stop:
      state = "Stop";
      break;
    case Step::Exit:
      state = "Exit";
      break;
    }
    if (entryLeft) {
      // 绘制锥桶坐标
      for (uint16_t i = 0; i < pointConeLeft.size(); i++) {
        circle(image, Point(pointConeLeft[i].y, pointConeLeft[i].x), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - LEFT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    } else {
      // 绘制锥桶坐标
      for (uint16_t i = 0; i < pointConeRight.size(); i++) {
        circle(image, Point(pointConeRight[i].y, pointConeRight[i].x), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - RIGHT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    putText(image, state, Point(COLSIMAGE / 2 - 10, 30),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

    putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
            CV_AA); // 显示锥桶距离
    if (_pointNearCone.x > 0)
      circle(image, Point(_pointNearCone.y, _pointNearCone.x), 5,
             Scalar(200, 200, 200), -1);

    if (levelCones > 0)
      line(image, Point(0, levelCones), Point(image.cols, levelCones),
           Scalar(255, 255, 255), 1);

    putText(image, to_string(indexDebug),
            Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
  }

private:
  bool again = false; // 第二次进入救援区标志
  double _distance = 0;
  int levelCones = 0; // 锥桶的平均高度
  POINT _pointNearCone;
  POINT pointHCone;
  vector<POINT> pointConeLeft;      // AI元素检测边缘点集
  vector<POINT> pointConeRight;     // AI元素检测边缘点集
  vector<POINT> lastPointsEdgeLeft; // 记录上一场边缘点集（丢失边）
  vector<POINT> lastPointsEdgeRight;

  vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
  vector<vector<POINT>> pathsEdgeRight;
  int indexDebug = 0;

  uint16_t counterSession = 0;  // 图像场次计数器
  uint16_t counterRec = 0;      // 标志检测计数器
  uint16_t counterExit = 0;     // 标志结束计数器
  uint16_t counterImmunity = 0; // 屏蔽计数器
  Mapping ipm =
      Mapping(Size(COLSIMAGE, ROWSIMAGE), Size(COLSIMAGEIPM, ROWSIMAGEIPM));
  /**
   * @brief 从AI检测结果中检索锥桶坐标集合
   *
   * @param predict AI检测结果
   * @return vector<POINT>
   */
  void searchCones(vector<PredictResult> predict) {
    pointConeLeft.clear();
    pointConeRight.clear();
    for (int i = 0; i < predict.size(); i++) {
      if (predict[i].type == LABEL_CONE) // 锥桶检测
      {
        if ((predict[i].x + predict[i].width / 2) < COLSIMAGE / 2)
          pointConeLeft.push_back(POINT(predict[i].y + predict[i].height,
                                        predict[i].x + predict[i].width));
        else
          pointConeRight.push_back(
              POINT(predict[i].y + predict[i].height, predict[i].x));
      }
    }
  }

  /**
   * @brief 搜索距离赛道左边缘最近的锥桶坐标
   *
   * @param pointsEdgeLeft 赛道边缘点集
   * @param predict AI检测结果
   * @return POINT
   */
  POINT getConeLeftDown(vector<POINT> pointsEdgeLeft,
                        vector<POINT> pointsCone) {
    POINT point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
      return point;

    for (int i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].x < pointsCone[i].x) {
        int row = pointsEdgeLeft[0].x - pointsCone[i].x;
        if (row > 0 && row < pointsEdgeLeft.size()) {
          int dis = pointsEdgeLeft[row].y - pointsCone[i].y;
          if (dis < disMin && pointsCone[i].x > ROWSIMAGE / 4 &&
              pointsCone[i].x > point.x) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }

    return point;
  }

  /**
   * @brief 搜索距离赛道右边缘最近的锥桶坐标
   *
   * @param pointsEdgeRight 赛道边缘点集
   * @param predict AI检测结果
   * @return POINT
   */
  POINT getConeRightDown(vector<POINT> pointsEdgeRight,
                         vector<POINT> pointsCone) {
    POINT point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeRight.size() < 10)
      return point;

    for (int i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeRight[pointsEdgeRight.size() - 1].x < pointsCone[i].x) {
        int row = pointsEdgeRight[0].x - pointsCone[i].x;
        if (row > 0 && row < pointsEdgeRight.size()) {
          int dis = pointsCone[i].y - pointsEdgeRight[row].y;
          if (dis < disMin && pointsCone[i].x > ROWSIMAGE / 4 &&
              pointsCone[i].x > point.x) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }

    return point;
  }

  /**
   * @brief 在俯视域由左边缘预测右边缘
   *
   * @param pointsEdgeLeft
   * @return vector<POINT>
   */
  vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeRight;
    if (pointsEdgeLeft.size() < 3)
      return pointsEdgeRight;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
    Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT startPoint = POINT(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
                pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
    prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
                pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
    prefictRight = Point2d(endIpm.x + offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT endPoint = POINT(endtIipm.y, endtIipm.x);

    // 补线
    vector<POINT> input = {startPoint, midPoint, endPoint};
    vector<POINT> repair = Bezier(0.05, input);

    for (int i = 0; i < repair.size(); i++) {
      if (repair[i].x >= ROWSIMAGE)
        repair[i].x = ROWSIMAGE - 1;

      else if (repair[i].x < 0)
        repair[i].x = 0;

      else if (repair[i].y >= COLSIMAGE)
        repair[i].y = COLSIMAGE - 1;
      else if (repair[i].y < 0)
        repair[i].y = 0;

      pointsEdgeRight.push_back(repair[i]);
    }

    return pointsEdgeRight;
  }

  /**
   * @brief 在俯视域由右边缘预测左边缘
   *
   * @param pointsEdgeRight
   * @return vector<POINT>
   */
  vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeLeft;
    if (pointsEdgeRight.size() < 3)
      return pointsEdgeLeft;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
    Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT startPoint = POINT(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y,
                pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
    prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);  // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y,
                pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
    prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT endPoint = POINT(endtIipm.y, endtIipm.x);

    // 补线

    vector<POINT> input = {startPoint, midPoint, endPoint};
    vector<POINT> repair = Bezier(0.05, input);

    for (uint16_t i = 0; i < repair.size(); i++) {
      if (repair[i].x >= ROWSIMAGE)
        repair[i].x = ROWSIMAGE - 1;

      else if (repair[i].x < 0)
        repair[i].x = 0;

      else if (repair[i].y >= COLSIMAGE)
        repair[i].y = COLSIMAGE - 1;
      else if (repair[i].y < 0)
        repair[i].y = 0;

      pointsEdgeLeft.push_back(repair[i]);
    }

    return pointsEdgeLeft;
  }

  /**
   * @brief 按照坐标点的y排序
   *
   * @param points
   * @return vector<int>
   */
  void pointsSortForY(vector<POINT> &points) {
    int n = points.size();
    bool flag = true;

    for (int i = 0; i < n - 1 && flag; i++) {
      flag = false;
      for (int j = 0; j < n - i - 1; j++) {
        if (points[j].y > points[j + 1].y) {
          POINT temp = points[j];
          points[j] = points[j + 1];
          points[j + 1] = temp;
          flag =
              true; // 每次循环i有修改，这里为true
                    // 如果跑了一次I没有发生交换的情况，说明已经排序完成，不需要再跑后面的i
        }
      }
    }
  }
};
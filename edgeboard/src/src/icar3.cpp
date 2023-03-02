#include "../include/common.hpp" //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp" //串口通信驱动
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区
#include "controlcenter.cpp"         //控制中心计算类
#include "motion.cpp"                //智能车运动控制类
#include "mapping.cpp"               //透视变换类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    Preprocess preprocess;    // 图像预处理类
    Motion motion;            // 运动控制类
    Tracking tracking;        // 赛道识别类
    Crossroad crossroad;      // 十字道路识别类
    Ring ring;                // 环岛识别类
    Bridge bridge;            // 坡道区检测类
    Parking parking;          // 停车区检测类
    Danger danger;            // 危险区检测类
    Rescue rescue;            // 救援区检测类
    Racing racing;            // 追逐区检测类
    ControlCenter ctrlCenter; // 控制中心计算类
    Display display(4);       // 初始化UI显示窗口
    VideoCapture capture;     // Opencv相机类

    // 目标检测类(AI模型文件)
    shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
    detection->score = motion.params.score; // AI检测置信度

    // USB转串口初始化： /dev/ttyUSB0
    shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
    int ret = uart->open();
    if (ret != 0)
    {
        printf("[Error] Uart Open failed!\n");
        return -1;
    }
    uart->startReceive(); // 启动数据接收子线程

    // USB摄像头初始化
    if (motion.params.debug)
        capture = VideoCapture(motion.params.video); // 打开本地视频
    else
        capture = VideoCapture(0); // 打开摄像头
    if (!capture.isOpened())
    {
        printf("can not open video device!!!\n");
        return 0;
    }
    capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
    capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
    // 等待按键发车
    if (!motion.params.debug)
    {
        printf("--------------[等待按键发车!]-------------------\n");
        //uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
        while (!uart->keypress)//按键1发车
            waitKey(300);
        while (ret < 10) // 延时3s
        {
            uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
            waitKey(300);
            ret++;
        }
        uart->keypress = false;
        //uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
        printf("---------------------[发车!]-------------------\n");
    }

    Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
    Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
    long preTime;
    Mat img;

    while (1)
    {
        if (motion.params.debug) // 综合显示调试UI窗口
            preTime = chrono::duration_cast<chrono::milliseconds>(
                          chrono::system_clock::now().time_since_epoch())
                          .count();
        if (!capture.read(img))
            continue;
        if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
            savePicture(img);


        //[02] 图像预处理
        if (!motion.params.debug)//如果不是debug模式,则旋转摄像头
            flip(img, img, -1);
        Mat imgCorrect = img.clone();         // 图像矫正
        Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化

    
        //[03] 启动AI推理
        detection->inference(imgCorrect);

        //[04] 赛道识别
        tracking.rowCutUp = motion.params.rowCutUp;         // 图像顶部切行（前瞻距离）
        tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
        tracking.trackRecognition(imgBinary);
        if (motion.params.debug) // 综合显示调试UI窗口
        {
            Mat imgTrack = imgCorrect.clone();
            tracking.drawImage(imgTrack); // 图像绘制赛道识别结果
            display.setNewWindow(2, "Track", imgTrack);

        }

        //[05] 停车区检测
        if (motion.params.parking)
        {
            if (parking.process(detection->results))
            {
                scene = Scene::ParkingScene;
                if (parking.countExit > 20)
                {
                    uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
                    sleep(1);
                    printf("-----> System Exit!!! <-----\n");
                    exit(0); // 程序退出
                }
            }
        }

        //[06] 救援区检测
        if ((scene == Scene::NormalScene || scene == Scene::RescueScene) &&
            motion.params.rescue) {
          if (rescue.process(tracking, detection->results))
            scene = Scene::RescueScene;
          else
            scene = Scene::NormalScene;
        }

        //[07] 追逐区检测
        if ((scene == Scene::NormalScene || scene == Scene::RacingScene) &&
            motion.params.racing) {
          if (racing.process(tracking, detection->results))
            scene = Scene::RacingScene;
          else
            scene = Scene::NormalScene;
        }

        //[08] 坡道区检测
        if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
            motion.params.bridge) {
          if (bridge.process(tracking, detection->results))
            scene = Scene::BridgeScene;
          else
            scene = Scene::NormalScene;
        }

        // [09] 危险区检测
        if ((scene == Scene::NormalScene || scene == Scene::DangerScene) &&
            motion.params.danger) {
          if (danger.process(tracking, detection->results)) {
            //uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
            scene = Scene::DangerScene;
          } else
            scene = Scene::NormalScene;
        }

        //[10] 十字道路识别与路径规划
        if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
            motion.params.cross)
        {
            if (crossroad.crossRecognition(tracking))
                scene = Scene::CrossScene;
            else
                scene = Scene::NormalScene;
        }

        //[11] 环岛识别与路径规划
        if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
            motion.params.ring)
        {
            if (ring.process(tracking, imgBinary))
                scene = Scene::RingScene;
            else
                scene = Scene::NormalScene;
        }

        //[12] 车辆控制中心拟合
        ctrlCenter.fitting(tracking);
        if (scene != Scene::RescueScene)
        {
            if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
            {
                uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
                sleep(1);
                printf("-----> System Exit!!! <-----\n");
                exit(0); // 程序退出
            }
        }

        //[13] 车辆运动控制(速度+方向)
        if (!motion.params.debug) // 非调试模式下
        {
            if ((scene == Scene::RescueScene && rescue.carStoping) || parking.park ||
                racing.carStoping) // 特殊区域停车
                motion.speed = 0;
            else if (scene == Scene::RescueScene && rescue.carExitting) // 倒车出库
                motion.speed = -motion.params.speedDown;
            else if (scene == Scene::RescueScene) // 减速
                motion.speedCtrl(true, true, ctrlCenter);
            else if (scene == Scene::BridgeScene) // 坡道速度
                motion.speed = motion.params.speedBridge;
            else
                motion.speedCtrl(true, false, ctrlCenter); // 车速控制

            motion.poseCtrl(ctrlCenter.controlCenter);       // 姿态控制（舵机）
            uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
            
        }
         motion.speedCtrl(true, false, ctrlCenter); // 车速控制

            motion.poseCtrl(ctrlCenter.controlCenter);       // 姿态控制（舵机）
            uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
        if (motion.params.debug)
        {
            // 帧率计算
            auto startTime = chrono::duration_cast<chrono::milliseconds>(
                                    chrono::system_clock::now().time_since_epoch())
                                    .count();

            Mat imgRes =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像

            switch (scene)
            {
            case Scene::NormalScene:
                break;
            case Scene::CrossScene:                    //[ 十字区 ]
                crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
                break;
            case Scene::RingScene:                //[ 环岛 ]
                ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
                break;
            case Scene::BridgeScene:                //[ 坡道区 ]
                bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
                circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
                        Scalar(40, 120, 250), -1);
                putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                        FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
                break;
            case Scene::DangerScene:      //[ 危险区 ]
                danger.drawImage(imgRes); // 图像绘制特殊赛道识别结果
                circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
                        Scalar(40, 120, 250), -1);
                putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                        FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
                break;
            case Scene::RescueScene:                //[ 救援区 ]
                rescue.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
                circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
                        Scalar(40, 120, 250), -1);
                putText(imgCorrect, "O", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                        FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
                break;
            case Scene::RacingScene:      //[ 追逐区 ]
                racing.drawImage(imgRes); // 图像绘制特殊赛道识别结果
                circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
                        Scalar(40, 120, 250), -1);
                putText(imgCorrect, "R", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                        FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
                break;
            case Scene::ParkingScene:      //[ 停车区 ]
                parking.drawImage(imgRes); // 图像绘制特殊赛道识别结果
                circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
                        Scalar(40, 120, 250), -1);
                putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                        FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
                break;
            default: // 常规道路场景：无特殊路径规划
                break;
            }

            display.setNewWindow(3, getScene(scene),
                                    imgRes); // 图像绘制特殊场景识别结果
            // detection->drawBox(imgCorrect); // 图像绘制AI结果
            ctrlCenter.drawImage(tracking,
                                    imgCorrect); // 图像绘制路径计算结果（控制中心）
            display.setNewWindow(4, "Ctrl", imgCorrect);
            display.show(); // 显示综合绘图
            waitKey(1);    // 等待显示
        }
        //[15] 状态复位
        // if (sceneLast != scene)
        // {
        //     if (scene == Scene::NormalScene)
        //         uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
        //     else
        //         uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
        // }
        sceneLast = scene; // 记录当前状态
        if (scene == Scene::DangerScene)
            scene = Scene::NormalScene;
        else if (scene == Scene::CrossScene)
            scene = Scene::NormalScene;

        //[16] 按键退出程序
        if (uart->keypress)
        {
            uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
            sleep(1);
            printf("-----> System Exit!!! <-----\n");
            exit(0); // 程序退出
        }

            // display.show(); // 显示综合绘图
            // waitKey(1);    // 等待显示
    }

    uart->close(); // 串口通信关闭
    capture.release();
    return 0;
}

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
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */
#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区
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
Mat ai_image;
bool start_ai=false;
bool start_uart=false;
std::thread ai_thread;
std::thread receive_thread;
std::thread send_thread;
std::mutex mut;
void thread_ai(std::shared_ptr<Detection> detection1){
  while(1){
    //std::cout<<"进去"<<std::endl;
    if(start_ai){
    mut.lock();
    Mat ii=ai_image.clone();
    start_ai=false;
    mut.unlock();
    detection1->inference(ii);
    //std::cout<<"进去2"<<std::endl;
    }
    else{
       waitKey(1);
    }
  }
}
void Send(std::shared_ptr<Uart> uart1,Motion& motion1) {
  // 启动串口接收子线程
  if (!uart1->isOpen) // 串口是否正常打开
    return;
  while (1) {
    if(start_uart){
      uart1->carControl(motion1.speed, motion1.servoPwm);
      //std::cout<<"进去了"<<motion1.servoPwm<<std::endl;
      start_uart=false;
    }
    else{
      //std::cout<<"进去了"<<std::endl;
      //uart1->receiveCheck();
      waitKey(1);
    }
  }
}
void Receive(std::shared_ptr<Uart> uart1) {
    if (!uart1->isOpen) // 串口是否正常打开
      return;
    // 启动串口接收子线程
      while (1) {
          uart1->receiveCheck(); // 串口接收校验
      }
}
int main(int argc, char const *argv[]) {
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
  Display display(2);       // 初始化UI显示窗口
  VideoCapture capture;     // Opencv相机类
  float speed=0.8;
  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度
  // USB转串口初始化： /dev/ttyUSB0
  shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  int ret = uart->open();
  if (ret != 0) {
    printf("[Error] Uart Open failed!\n");
    return -1;
  }
  //uart->startReceive(); // 启动数据接收子线程
  receive_thread=std::thread(Receive,uart);
  if (!motion.params.debug) {  
    ai_thread = std::thread(thread_ai, detection); // 在这里初始化 ai_thread 对象  
  }  
  //send_thread=std::thread(Send,uart,std::ref(motion));
  // USB摄像头初始化
  if (0)
    capture = VideoCapture(motion.params.video); // 打开本地视频
  else
    capture = VideoCapture(0,cv::CAP_V4L2); // 打开摄像头
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }
  else{
    printf("can open video\n");
  }
  VideoWriter vw;
  int fourcc = vw.fourcc('M','J','P','G');//设置摄像头编码方式
  capture.set(CAP_PROP_FOURCC,fourcc);
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
  capture.set(CAP_PROP_FPS,120);
  std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();  
  int frame_count = 0;  
  double fps = 0.0; 
  // 等待按键发车
  if (0) {
    printf("--------------[等待按键发车!]-------------------\n");
    //uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
    while (!uart->keypress)
      waitKey(300);
    while (ret < 10) // 延时3s
    {
      uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
      waitKey(300);
      ret++;
    }
    uart->keypress = false;
    //uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
  }

  // 初始化参数
  Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
  long preTime;
  Mat img;
  while (1) {
    //[01] 视频源读取
    if (motion.params.debug_fps) // 综合显示调试UI窗口
      preTime = chrono::duration_cast<chrono::milliseconds>(
                    chrono::system_clock::now().time_since_epoch())
                    .count();
    if (!capture.read(img))
       continue;
    if (0)//如果不是debug模式,则旋转摄像头
      flip(img, img, -1);
    // //[02] 图像预处理
    Mat imgCorrect = img.clone();         // 图像矫正
    mut.lock();
    ai_image=img.clone();
    start_ai=true;
    mut.unlock();
    Mat imgBinary;
    imgBinary = preprocess.binaryzation(img); // 图像二值化
    // cv::warpPerspective(imgBinary, imgBinary, H_inv, img.size());
    // cv::warpPerspective(imgCorrect, imgCorrect, H_inv, img.size());  
    // //[03] 启动AI推理
    if(motion.params.debug)
      detection->inference(img);

    // //[04] 赛道识别
    tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（前瞻距离）
    tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary);
    // // Mat ii=imgCorrect.clone();
    if (0) // 综合显示调试UI窗口
    {
      Mat imgTrack = imgCorrect.clone();
      tracking.drawImage(imgTrack); // 图像绘制赛道识别结果
      display.setNewWindow(2, "Track", imgTrack);
            display.show(); // 显示综合绘图
      waitKey(10);    // 等待显示
    }
    //[05] 停车区检测
    if (motion.params.parking) {
      if (parking.process(detection->results)) {
        scene = Scene::ParkingScene;
        if (parking.countExit > 5) {
          uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
          sleep(1);
          printf("-----> System Exit!!! <-----\n");
          exit(0); // 程序退出
        }
      }
    }

    // //[06] 救援区检测
    if ((scene == Scene::NormalScene || scene == Scene::RescueScene) &&
        motion.params.rescue) {
      if (rescue.process(tracking, detection->results))
        scene = Scene::RescueScene;
      else
        scene = Scene::NormalScene;
    }
    // // // [09] 危险区检测
    if ((scene == Scene::NormalScene || scene == Scene::DangerScene) &&
        motion.params.danger) {
      if (danger.process(tracking, detection->results)) {
        //uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
        scene = Scene::DangerScene;
      } else
        scene = Scene::NormalScene;
    }

    // //[11] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      if (ring.process(tracking, imgCorrect))
        scene = Scene::RingScene;
      else
        scene = Scene::NormalScene;
    }
    // Mat ii=imgCorrect.clone();
    //      ctrlCenter.drawImage(tracking,
    //                         ii); 
    // savePicture(ii);
  //elements(tracking);
    //[12] 车辆控制中心拟合
    float prospect=0;
    if(speed==motion.params.speedHigh)
      prospect= motion.params.prospect;
    else if(scene ==Scene::RingScene)
      prospect= motion.params.prospect+0.2;
    else
      prospect= motion.params.prospect+0.1;
    ctrlCenter.fitting(tracking,prospect,imgCorrect,scene);
    // if (scene != Scene::RescueScene) {
    //   if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
    //   {
    //     uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
    //     sleep(1);
    //     printf("-----> System Exit!!! <-----\n");
    //     exit(0); // 程序退出
    //   }
    // }
    //[13] 车辆运动控制(速度+方向)
      //std::cout<<"2"<<std::endl;
      //std::cout<<abs(ctrlCenter.sigmaCenter);
    if ((scene == Scene::RescueScene && rescue.carStoping) || parking.park ||
        racing.carStoping) // 特殊区域停车
      motion.speed = 0;
    else if (scene == Scene::RescueScene && rescue.carExitting) // 倒车出库
      motion.speed = -motion.params.speedDown;
    else if(scene == Scene::DangerScene)
      motion.speed = motion.params.speedDown-0.1;
    else if(scene ==Scene::RingScene)
      motion.speed = motion.params.speedLow-0.3;
    else if (scene == Scene::RescueScene) // 减速
      motion.speedCtrl(true, true, ctrlCenter);
    else
      motion.speedCtrl(true, false, ctrlCenter); // 车速控制
    speed=motion.speed_control(motion.speed,speed);
    motion.poseCtrl(ctrlCenter.controlCenter); // 姿态控制（舵机
    // mut.lock();
    // start_uart=true;
    // mut.unlock();
    std::cout<<"速度"<<speed;
    uart->carControl(speed, motion.servoPwm); // 串口通信控制车辆
    //std::cout<<"速度"<<
    if(motion.speed == -motion.params.speedDown){
      waitKey(8);
    }
    //std::cout<<motion.speed<<"\t";
    //[15] 状态复位
    // if (sceneLast != scene) {
    //   if (scene == Scene::NormalScene)
    //     //uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
    //   //else
    //     //uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
    // }
    // cv::imshow("1",img);
    // if (cv::waitKey(1) == 'q') {  
    //         break;  
    //     }

    //[14] 综合显示调试UI窗口
    if (motion.params.debug) {
      // 帧率计算
      display.setNewWindow(2, "Binary", imgBinary);
      Mat imgRes =
          Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像

      switch (scene) {
      case Scene::NormalScene:
        break;
      case Scene::CrossScene:                  //[ 十字区 ]
        crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        break;
      case Scene::RingScene:              //[ 环岛 ]
        ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        break;
      case Scene::BridgeScene:              //[ 坡道区 ]
        bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::DangerScene:    //[ 危险区 ]
        danger.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::RescueScene:              //[ 救援区 ]
        rescue.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "O", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::RacingScene:    //[ 追逐区 ]
        racing.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "R", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::ParkingScene:    //[ 停车区 ]
        parking.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      default: // 常规道路场景：无特殊路径规划
        break;
      }

      // display.setNewWindow(1, getScene(scene),
      //                      imgRes);   // 图像绘制特殊场景识别结果
      detection->drawBox(imgCorrect); // 图像绘制AI结果
      ctrlCenter.drawImage(tracking,
                           imgCorrect); // 图像绘制路径计算结果（控制中心）
      display.setNewWindow(1, "Ctrl", imgCorrect);
      display.show(); // 显示综合绘图
      waitKey(8);    // 等待显示
    }
    // detection->drawBox(imgCorrect); 
    // ctrlCenter.drawImage(tracking,
    //                        imgCorrect); // 图像绘制路径计算结果（控制中心
    //std::cout<<scene<<std::endl;
    sceneLast = scene; // 记录当前状态
    // if (scene == Scene::DangerScene)
    //   scene = Scene::NormalScene;-
   if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;
    
    //[16] 按键退出程序
    if (uart->keypress) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      sleep(1);
      printf("-----> System Exit!!! <-----\n");
      exit(0); // 程序退出
    }
    if(motion.params.debug_fps){
        auto startTime = chrono::duration_cast<chrono::milliseconds>(
                          chrono::system_clock::now().time_since_epoch())
                          .count();
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();  
        std::chrono::duration<double> elapsed_seconds = current_time - prev_time;  
        if (elapsed_seconds.count() >= 1)  
        {  
            fps = frame_count / elapsed_seconds.count();
            printf(">> FrameTime: %ldms | %.2ffps \t", startTime - preTime,1000.0 / (startTime - preTime));
            std::cout << "FPS: " << fps << std::endl;  
            // 重置时间点和帧数  
            prev_time = current_time;  
            frame_count = 0;  
        }  
        frame_count++;
    }
    if (motion.params.saveImg) // 存储原始图像
      savePicture(imgCorrect);
  }
  start_ai=false;
  if (ai_thread.joinable()) { // 检查线程是否已被创建（即可连接）  
    ai_thread.join(); // 等待线程结束  
  }
  receive_thread.join();
  //send_thread.join();
  uart->close(); // 串口通信关闭
  capture.release();
  return 0;
}

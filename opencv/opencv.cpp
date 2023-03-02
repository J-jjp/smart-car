#include <opencv2/opencv.hpp>  
#include <iostream>  
#include <chrono>  
  
int main(int argc, char** argv)  
{  

  
    cv::VideoCapture cap(0,cv::CAP_V4L2); // 打开视频文件或摄像头  
  
    if (!cap.isOpened())  
    {  
        std::cerr << "Error opening video file or camera" << std::endl;  
        return -1;  
    }  
  
    // 初始化时间变量  
    std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();  
    int frame_count = 0;  
    double fps = 0.0;  
    cv::VideoWriter vw;
    int fourcc = vw.fourcc('M','J','P','G');//设置摄像头编码方式
    cap.set(cv::CAP_PROP_FOURCC,fourcc);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 设置图像分辨率
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 设置图像分辨率
    cap.set(cv::CAP_PROP_FPS,120);
   double change_un_Mat[3][3] ={{0.751482,-0.361073,23.766578},{-0.011420,0.199386,13.298054},{-0.000013,-0.001281,0.908921}};
            cv::Mat H(3, 3, CV_64F); // 创建一个3x3的双精度浮点数矩阵  
        for (int i = 0; i < 3; ++i) {  
            for (int j = 0; j < 3; ++j) {  
                H.at<double>(i, j) = change_un_Mat[i][j];  
            }  
        }
            cv::Mat H_inv;  
        try {  
            H_inv = H.inv(); // 尝试计算逆矩阵  
        } catch (const cv::Exception& e) {  
            std::cerr << "Error computing inverse homography matrix: " << e.what() << std::endl;  
            return -1;  
        }
    while (true)  
    {  
        cv::Mat frame;  
        bool success = cap.read(frame); // 读取一帧  

        if (!success)  
        {  
            std::cout << "End of video" << std::endl;  
            break;  
        }  
        //std::cout<<frame.cols<<"\t"<<frame.rows<<std::endl; 
        cv::Mat dstImage;  
        dstImage.create(frame.size(), frame.type());  
  
    // 使用warpPerspective函数和逆矩阵H_inv进行逆透视变换  
        cv::warpPerspective(frame, dstImage, H_inv, frame.size());  
  
        // 计算帧率  
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();  
        std::chrono::duration<double> elapsed_seconds = current_time - prev_time;  

        // 每秒更新一次帧率  
        if (elapsed_seconds.count() >= 1)  
        {  
            fps = frame_count / elapsed_seconds.count();  
            std::cout << "FPS: " << fps << std::endl;  
  
            // 重置时间点和帧数  
            prev_time = current_time;  
            frame_count = 0;  
        }  
  
        frame_count++;  
  
        // 显示帧（可选）  
          cv::imshow("F", dstImage);
        cvtColor(dstImage, dstImage, cv::COLOR_BGR2GRAY); // RGB转灰度图
        threshold(dstImage, dstImage, 0, 255, cv::THRESH_OTSU); // OTSU二值化方法
                cv::imshow("Frame", dstImage);  
        // 退出条件，例如按下'q'键  
        char c = (char)cv::waitKey(1);  
        if (c == 'q' || c == 27)  
            break;  
    }  
  
    cap.release(); // 释放视频捕获对象  
    cv::destroyAllWindows(); // 关闭所有OpenCV窗口  
  
    return 0;  
}
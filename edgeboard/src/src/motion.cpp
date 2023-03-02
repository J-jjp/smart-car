#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter.cpp"
#include "FuzzyPID.cpp"
using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion
{
private:
    int countShift = 0; // 变速计数器
    std::vector<double> errors;
    //FuzzyPID fuzzy;

public:
    /**
     * @brief 初始化：加载配置文件
     *
     */
    Motion()
    {
        string jsonPath = "../src/config/config.json";
        std::ifstream config_is(jsonPath);//打开json文件
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath
                      << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        speed = params.speedLow;
        //FuzzyPID pid;
    };

    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float speedLow = 0.8;                              // 智能车最低速
        float speedHigh = 0.8;                             // 智能车最高速
        float speedBridge = 0.6;                           // 坡道速度
        float speedDown = 0.5;                             // 特殊区域降速速度
        float runP1 = 0;                                 // 一阶比例系数：直线控制量
        float runP2 = 0;
        float runP3 = 0.0;                                 // 三阶比例系数：弯道控制量
        float runP4=0;
        float runP=0.0;
        float prospect=0.0;
        float turnD1 = 0;
        float turnD2= 0;
        float turnD3 = 0;                                 // 一阶微分系数：转弯控制量
        float turnD=0;
        bool debug = true;                                // 调试模式使能
        bool saveImg = false;                              // 存图使能
        uint16_t rowCutUp = 10;                            // 图像顶部切行
        uint16_t rowCutBottom = 10;                        // 图像低部切行
        bool bridge = true;                                // 坡道区使能
        bool danger = true;                                // 危险区使能
        bool rescue = true;                                // 救援区使能
        bool racing = true;                                // 追逐区使能
        bool parking = true;                               // 停车区使能
        bool debug_fps=true;                               //帧率测试使能
        bool debug_uart=true;                              //串口发送使能
        bool ring = true;                                  // 环岛使能
        bool cross = true;                                 // 十字道路使能
        float score = 0.5;                                 // AI检测置信度
        string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
        string video = "../res/samples/sample.mp4";          // 视频路径
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge, speedDown, runP1, runP2, runP3,runP4,
                                       runP,prospect,turnD1,turnD2,turnD3, turnD, debug, saveImg, rowCutUp, rowCutBottom, bridge, danger,
                                       rescue, racing, parking, debug_fps,debug_uart,ring, cross, score, model, video); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.3;               // 发送给电机的速度
    int pwm_Diff;
    int errp_ppre = 0;  
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void poseCtrl(int controlCenter)
    {
        float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
                //std::cout<<error<<std::endl;
        static int errorLast = 0;                    // 记录前一次的偏差    
        if (abs(error - errorLast) > COLSIMAGE / 10)
        {
            //std::cout<<"进去了"<<error<<"\t"<<errorLast<<std::endl;
            error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;
        }
        // 每收集10个误差样本后进行一次PID参数的自适应调
        // if(speed==params.speedLow){
        //     errors.push_back(std::abs(error));
        //     if (errors.size() >= 10) {
        //         adaptPIDGains();
        //     }
        // }
        // std::cout<<"\tp"<<params.runP1;
        // std::cout<<"\tp2"<<params.runP2;
        // std::cout<<"\td"<<params.turnD1<<std::endl;
        if(speed==params.speedDown){
            params.runP = abs(error) * params.runP4 + params.runP3;
            int pwmDiff = (error * params.runP) + (error - errorLast) * params.turnD2;
            if(abs(pwmDiff)>300){
                if(pwmDiff>0)
                    pwmDiff=300;
                else
                    pwmDiff=-300;
            }
             servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff);
        }
        else{ 
            params.runP = abs(error) * params.runP2 + params.runP1;
            int pwmDiff = (error * params.runP) + (error - errorLast) * params.turnD1;
            if(abs(pwmDiff)>450){
                if(pwmDiff>0)
                    pwmDiff=450;
                else
                    pwmDiff=-450;
            }
            servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); 
        }
        //std::cout<<servoPwm<<std::endl;
        errp_ppre=errorLast;
        errorLast = error;
        
    }

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedCtrl(bool enable, bool slowDown, ControlCenter control)
    {
        // 控制率
        uint8_t controlLow = 0;   // 速度控制下限
        uint8_t controlMid = 5;   // 控制率
        uint8_t controlHigh = 10; // 速度控制上限

        if (slowDown)
        {
            countShift = controlLow;
            speed = params.speedDown;
        }
        else if (enable) // 加速使能
        {
            if (control.centerEdge.size() < 10)
            {
                speed = params.speedLow;
                countShift = controlLow;
                return;
            }
            if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
            {
                speed = params.speedLow;
                countShift = controlLow;
                return;
            }
            if (abs(control.sigmaCenter) <100.0)
            {
                countShift++;
                if (countShift > controlHigh)
                    countShift = controlHigh;
            }
            else
            {
                countShift--;
                if (countShift < controlLow)
                    countShift = controlLow;
            }

            if (countShift > controlMid)
                speed = params.speedHigh;
            else
                speed = params.speedLow;
        }
        else
        {
            countShift = controlLow;
            speed = params.speedLow;
        }
    }
    void adaptPIDGains() {
        double errorSum = std::accumulate(errors.begin(), errors.end(), 0.0);
        double averageError = errorSum / errors.size();
        double maxError = *std::max_element(errors.begin(), errors.end());
        double minError = *std::min_element(errors.begin(), errors.end());

        // 调整 params.runP1
        if (averageError > 50) {
            params.runP1 *= 1.05;  // 如果平均误差较大，增加 runP1
        } else if (averageError < 20) {
            params.runP1 *= 0.95;  // 如果平均误差较小，减少 runP1
        }

        // 调整 params.runP2
        if (maxError > 50) {
            params.runP2 *= 1.1;  // 如果最大误差过大，增加 runP2
        } else if (minError < 20) {
            params.runP2 *= 0.9;  // 如果最小误差过小，减少 runP2
        }

        // 调整 params.turnD1
        if (averageError > 40) {
            params.turnD1 *= 1.025;  // 如果平均误差较大，增加 turnD1
        } else if (averageError < 10) {
            params.turnD1 *= 0.975;  // 如果平均误差较小，减少 turnD1
        }

        errors.clear();  // 清空误差列表以便重新收集数据
    }

};

/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*运动方向控制序列*/
enum ConquerorCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum ConquerorCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

/*控制管理成员*/
struct Application_xxx
{
  ConquerorCarMotionControl Motion_Control;
  ConquerorCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_ConquerorCarxxx0;

bool ApplicationFunctionSet_ConquerorCarLeaveTheGround(void);
void ApplicationFunctionSet_ConquerorCarLinearMotionControl(ConquerorCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_ConquerorCarMotionControl(ConquerorCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/
  }
  delay(2000);
  Application_ConquerorCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
}


/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限
*/
static void ApplicationFunctionSet_ConquerorCarLinearMotionControl(ConquerorCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_ConquerorCarMotionControl(ConquerorCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


    Kp = 2;
    UpperLimit = 180;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_ConquerorCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      ApplicationFunctionSet_ConquerorCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 10;
    break;
  }
}

/*
  避障功能
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    static unsigned long timestamp = 0;
    static bool en = false;
    static uint8_t en_ULTRASONIC_Get = 0;
    static uint8_t is_ULTRASONIC_Get = 1;

    if (Car_LeaveTheGround == false) //检测小车是否离开地面
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
      en = false;
      return;
    }
    else
    {
      if (is_ULTRASONIC_Get != en_ULTRASONIC_Get || en == false)
      {
        if (en != false)
        { //为了保证超声波数据采集的准确性、运动装置需保持静止（即：测距操作时停止对小车左右运动控制）
          ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
        }
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/);
        en_ULTRASONIC_Get = is_ULTRASONIC_Get;
      }
#if _Test_print
      Serial.print("UltrasoundData_cm=");
      Serial.println(UltrasoundData_cm);
#endif
      {
        if (function_xxx(UltrasoundData_cm, 0, ObstacleDetection)) //前方 ObstacleDetection cm内有障碍物？
        {
          //获取时间戳 timestamp
          if (en == false || millis() - timestamp > 3100)
          {
            en = true;
            timestamp = millis();
          }
          if (function_xxx((millis() - timestamp), 0, 500)) //-
          {
            is_ULTRASONIC_Get = 1;
            ApplicationFunctionSet_ConquerorCarMotionControl(Right, 250);
          }
          else if (function_xxx((millis() - timestamp), 500, 600))
          {
            is_ULTRASONIC_Get = 2;
          }
          else if (function_xxx((millis() - timestamp), 600, 1600)) //+
          {
            ApplicationFunctionSet_ConquerorCarMotionControl(Left, 250);
          }
          else if (function_xxx((millis() - timestamp), 1600, 1700))
          {
            is_ULTRASONIC_Get = 3;
          }
          else if (function_xxx((millis() - timestamp), 1700, 2200)) //0
          {
            ApplicationFunctionSet_ConquerorCarMotionControl(Right, 250);
          }
          else if (function_xxx((millis() - timestamp), 2200, 2700)) //
          {
            ApplicationFunctionSet_ConquerorCarMotionControl(Backward, 250);
          }
          else if (function_xxx((millis() - timestamp), 2700, 3000)) //0
          {
            ApplicationFunctionSet_ConquerorCarMotionControl(Left, 250);
          }
          else if (function_xxx((millis() - timestamp), 3000, 3100)) //0
          {
            is_ULTRASONIC_Get = 4;
          }
        }
        else //只有前方有效控制范围内无障碍，则永远保持向前直行运动
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(Forward, 250);
          en = false;
        }
      }
    }
  }
}

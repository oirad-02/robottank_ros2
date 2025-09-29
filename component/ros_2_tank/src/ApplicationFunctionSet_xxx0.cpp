
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

//#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;
/*f(x) int */
static boolean function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*Movement direction control sequence*/
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

/*Mode Control Sequence*/
enum ConquerorCarFunctionalModel
{
  Standby_mode,           /*Idle Mode*/
  TraceBased_mode,        /*Tracking Mode*/
  ObstacleAvoidance_mode, /*Obstacle Avoidance Mode*/
  Follow_mode,            /*Follow Mode*/
  Rocker_mode,            /*Joystick Mode*/
  CMD_inspect,
  CMD_Programming_mode,                   /*Programming Mode*/
  CMD_ClearAllFunctions_Standby_mode,     /*Clear All Functions: Enter Standby Mode*/
  CMD_ClearAllFunctions_Programming_mode, /*Clear all functions: enter programming mode*/
  CMD_MotorControl,                       /*Motor control mode*/
  CMD_CarControl_TimeLimit,               /*Car direction control: time-limited mode*/
  CMD_CarControl_NoTimeLimit,             /*Car direction control: no time-limited mode*/
  CMD_MotorControl_Speed,                 /*Motor control: Speed control mode*/
  CMD_ServoControl,                       /*Servo control: Mode*/
  CMD_LightingControl_TimeLimit,          /*Lighting control: Mode*/
  CMD_LightingControl_NoTimeLimit,        /*Lighting control: Mode*/
};

struct Vector3 {
  float x, y, z;
  
  Vector3(float x = 0, float y = 0, float z = 0) 
    : x(x), y(y), z(z) {}
};

Vector3 myVec(1.5, 2.0, 3.5);

/*Control management members*/
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
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    /*Clear serial port buffer...*/
  }
  Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
}

/*ITR20001 Detect whether the trolley has left the ground*/
static bool ApplicationFunctionSet_ConquerorCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}
/*
  Linear motion control:
  direction: Direction selection (forward/backward)
  directionRecord: Direction record (updates direction position data when first entering this function, i.e., yaw)
  speed: Input speed (0–255)
  Kp: Position error amplification ratio constant term (improves the response of the position recovery state; modified according to different motion working modes when input)
  UpperLimit: Maximum output control limit
*/
static void ApplicationFunctionSet_ConquerorCarLinearMotionControl(ConquerorCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; 
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
  //Add proportional constant Kp
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
  if (direction == Forward) 
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}

static void ApplicationFunctionSet_ConquerorCarLinearMotionControl_cmdvel(
    float linear_x, float angular_z, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
    static float Yaw; 
    static float yaw_So = 0;
    static uint8_t en = 110;
    static unsigned long is_time;

    if (en != directionRecord || millis() - is_time > 10)
    {
        AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        is_time = millis();
    }
    if (en != directionRecord)
    {
        en = directionRecord;
        yaw_So = Yaw;
    }

    float yaw_error = Yaw - yaw_So;

    // Basisgeschwindigkeit plus Drehanteil und Yaw-Korrektur
    int R = (int)(yaw_error * Kp) + speed + (int)(angular_z * UpperLimit);
    int L = (int)(-yaw_error * Kp) + speed - (int)(angular_z * UpperLimit);

    R = constrain(R, 10, UpperLimit);
    L = constrain(L, 10, UpperLimit);

    boolean dir_R = (R >= 0) ? direction_just : direction_back;
    boolean dir_L = (L >= 0) ? direction_just : direction_back;

    uint8_t speed_R = abs(R);
    uint8_t speed_L = abs(L);

    AppMotor.DeviceDriverSet_Motor_control(dir_R, speed_R, dir_L, speed_L, control_enable);
}



/*
  Motion control:
  1# Direction: Forward (1), Backward (2), Left Forward (3), Right Forward (4), Backward Left (5), Backward Right (6)
  2# Speed (0–255)
*/
static void ApplicationFunctionSet_ConquerorCarMotionControl(ConquerorCarMotionControl direction, uint8_t is_speed)
{
  //ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  //Control mode requiring linear motion adjustment (in the following working motion modes, the cart is prone to positional deviation when moving forward and backward, and the motion does not achieve the desired linear direction, so control adjustment is required)
  switch (Application_ConquerorCarxxx0.Functional_Mode)
  {
  case Rocker_mode:
    Kp = 10;
    UpperLimit = 255;
    break;
  case ObstacleAvoidance_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case Follow_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_TimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_NoTimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  default:
    Kp = 10;
    UpperLimit = 255;
    break;
  }
  switch (direction)
  {
  case /* constant-expression */ Forward:
    
    if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,/*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //Enter direction position approach control loop processing when moving forward
      ApplicationFunctionSet_ConquerorCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,/*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //Enter direction position approach control loop processing when reversing
      ApplicationFunctionSet_ConquerorCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:

    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,/*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,/*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,/*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,/*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,/*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,/*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,/*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 10;
    break;
  }
}

/*
 Sensor data update: Local update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{
  { /*Voltage status update*/
    static unsigned long VoltageData_time = 0;
    static uint8_t VoltageData_number = 1;
    if (millis() - VoltageData_time > 10)  //Collect and update once every 10 ms
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        if (VoltageData_number == 250) //Continuously check the latest voltage value multiple times...
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

 

  {  /*R Trace status update*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 Detect whether the cart has left the ground
    ApplicationFunctionSet_ConquerorCarLeaveTheGround();
  }

  
}
/*
  Power-on action requirements:
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void)
{
  Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
}

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  switch (is_LightingSequence)
  {
  case 0:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 1: /*Left*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 2: /*Previous*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 3: /*Right*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 4: /*After*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 5: /*Middle*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  default:
    break;
  }
}

/*RBG_LED collection*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) //Low voltage?
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
    case /* constant-expression */ 0 ... 49:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 50 ... 99:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
    case /* constant-expression */ 100 ... 149:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 150 ... 199:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 200 ... 249:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 250 ... 299:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 300 ... 349:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 350 ... 399:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 400 ... 449:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 450 ... 499:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    default:
      break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_ConquerorCarxxx0.Functional_Mode) //Act on mode control sequence
    {
    case /* constant-expression */ Standby_mode:
      {
        if (VoltageDetectionStatus == true)
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay(30);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay(30);
        }
        else
        {
          static uint8_t setBrightness = 0;
          static boolean et = false;
          static unsigned long time = 0;

          if ((millis() - time) > 30)
          {
            time = millis();
            if (et == false)
            {
              setBrightness += 1;
              if (setBrightness == 80)
                et = true;
            }
            else if (et == true)
            {
              setBrightness -= 1;
              if (setBrightness == 1)
                et = false;
            }
          }
          // AppRBG_LED.leds[1] = CRGB::Blue;
          AppRBG_LED.leds[0] = CRGB::Violet;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
      }
      break;
    case /* constant-expression */ CMD_Programming_mode:
      {
      }
      break;
    case /* constant-expression */ TraceBased_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Green);
      }
      break;
    case /* constant-expression */ ObstacleAvoidance_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      }
      break;
    case /* constant-expression */ Follow_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
      }
      break;
    case /* constant-expression */ Rocker_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Violet);
      }
      break;
    default:
      break;
    }
  }
}

/*Joystick*/
void ApplicationFunctionSet::ApplicationFunctionSet_Joystick(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == Rocker_mode)
  {
    ApplicationFunctionSet_ConquerorCarMotionControl(Application_ConquerorCarxxx0.Motion_Control /*direction*/, 255 /*speed*/);
  }
}

/*Tracking*/
void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //Did the car leave the ground?
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
      return;
    }
    int getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    int getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    int getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
#if _Test_print
    static unsigned long print_time = 0;
    if (millis() - print_time > 500)
    {
      print_time = millis();
      Serial.print("ITR20001_getAnaloguexxx_L=");
      Serial.println(getAnaloguexxx_L);
      Serial.print("ITR20001_getAnaloguexxx_M=");
      Serial.println(getAnaloguexxx_M);
      Serial.print("ITR20001_getAnaloguexxx_R=");
      Serial.println(getAnaloguexxx_R);
    }
#endif
    if (function_xxx(getAnaloguexxx_M, TrackingDetection_S, TrackingDetection_E))
    {
      /*Control the rotation of the left and right motors: achieve uniform straight-line movement*/
      ApplicationFunctionSet_ConquerorCarMotionControl(Forward, 200);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_R, TrackingDetection_S, TrackingDetection_E))
    {
      /*Control left and right motor rotation: front right*/
      ApplicationFunctionSet_ConquerorCarMotionControl(Right, 200);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_L, TrackingDetection_S, TrackingDetection_E))
    {
      /*Control left and right motor rotation: front left*/
      ApplicationFunctionSet_ConquerorCarMotionControl(Left, 200);
      timestamp = true;
      BlindDetection = true;
    }
    else //When not on the black line
    {
      if (timestamp == true) //Get timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
      }
      /*Blind Detection*/
      if ((function_xxx((millis() - MotorRL_time), 0, 200) || function_xxx((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_ConquerorCarMotionControl(Right, 250);
      }
      else if (((function_xxx((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_ConquerorCarMotionControl(Left, 250);
      }
      else if ((function_xxx((millis() - MotorRL_time), 3000, 3500))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

/*
  Obstacle avoidance function
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    static unsigned long timestamp = 0;
    static bool en = false;
    static uint8_t en_ULTRASONIC_Get = 0;
    static uint8_t is_ULTRASONIC_Get = 1;

    if (Car_LeaveTheGround == false) //Detect whether the car has left the ground
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
        { //To ensure the accuracy of ultrasonic data collection, the motion device must remain stationary (i.e., stop controlling the left and right movement of the cart during the distance measurement operation).
          ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
        }
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(const_cast<uint16_t*>(&UltrasoundData_cm));
        en_ULTRASONIC_Get = is_ULTRASONIC_Get;
      }
#if _Test_print
      Serial.print("UltrasoundData_cm=");
      Serial.println(UltrasoundData_cm);
#endif
      {
        if (function_xxx(UltrasoundData_cm, 0, ObstacleDetection))  //ObstacleDetection Is there an obstacle within cm ahead?
        {
           //Get timestamp
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
        else //Only when there are no obstacles within the effective control range ahead, will it always maintain forward motion.
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(Forward, 250);
          en = false;
        }
      }
    }
  }
}

/*
  Follow pattern:
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void)
{

  if (Application_ConquerorCarxxx0.Functional_Mode == Follow_mode)
  {
    static bool en = false;
    static unsigned long timestamp = 0;
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
      return;
    }
    else
    {
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(const_cast<uint16_t*>(&UltrasoundData_cm) /*out*/);
#if _Test_print
      Serial.print("UltrasoundData_cm=");
      Serial.println(UltrasoundData_cm);
#endif

      if (function_xxx(UltrasoundData_cm, 0, ObstacleDetection))  //ObstacleDetection cm ahead, contact detected? (Equivalent to obstacle avoidance mode obstacle)
      {
         //Keep moving forward in a straight line
        ApplicationFunctionSet_ConquerorCarMotionControl(Forward, 250);
        en = false;
      }
      else
      {
        //Get timestamp
        if (en == false)
        {
          en = true;
          timestamp = millis();
        }
        if (function_xxx((millis() - timestamp), 0, 500) || function_xxx((millis() - timestamp), 1000, 1500) || function_xxx((millis() - timestamp), 2000, 2500) || function_xxx((millis() - timestamp), 3000, 3500)) //
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
        }
        else if (function_xxx((millis() - timestamp), 500, 1000)) //-
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(Right, 255);
        }
        else if (function_xxx((millis() - timestamp), 1500, 2000) || function_xxx((millis() - timestamp), 2500, 3000)) //+
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(Left, 255);
        }
        else if (function_xxx((millis() - timestamp), 3500, 4000)) //+
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(Right, 255);
        }
        else if (function_xxx((millis() - timestamp), 4000, 4500)) //+
        {
          ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
        }
      }
    }
  }
}
/*Servo control*/
void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 90;
  static int y_angle = 90;
  uint8_t is_Servo = Set_Servo; //Prevent optimization

  switch (is_Servo)//Wichtig
  {
  case 1 ... 2:
  {
    if (1 == is_Servo)
    {
      y_angle -= 10;
    }
    else if (2 == is_Servo)
    {
      y_angle += 10;
    }
    if (y_angle <= 30) //Lower limit control
    {
      y_angle = 30;
    }
    if (y_angle >= 110) //Upper and lower limit control
    {
      y_angle = 110;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == is_Servo)
    {
      z_angle += 10;
    }
    else if (4 == is_Servo)
    {
      z_angle -= 10;
    }

    if (z_angle <= 0) //Lower limit control
    {
      z_angle = 0;
    }
    if (z_angle >= 180) //Upper and lower limit control
    {
      z_angle = 180;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ 90);
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 90);
    break;
  default:
    break;
  }
}
/*Standby*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_ConquerorCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    if (true == is_ED) //Zeroing the yaw raw data (make sure the cart is placed on a stationary plane!)
    {
      static unsigned long timestamp; //Get timestamp
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_ConquerorCarLeaveTheGround())
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

/* 
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Begin:CMD
 * wichtig
*/

void ApplicationFunctionSet::CMD_inspect_xxx0(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_inspect)
  {
    Serial.println("CMD_inspect");
    delay(100);
  }
}
/*
  N1: Command
  CMD Mode: Motion Mode <Motor Control> Receive and execute single-direction motor drive based on control commands from the APP end
  Input: uint8_t is_MotorSelection, Motor Selection   1 Left  2 Right  0 All
        uint8_t is_MotorDirection, Motor Direction  1 Forward  2 Reverse  0 Stop
        uint8_t is_MotorSpeed, Motor Speed   0-250
        No time limit
*/
void ApplicationFunctionSet::CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == is_MotorDirection)
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (is_MotorSelection) //Motor selection
      {
      case 0:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A, /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControl_xxx0(void)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection)
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (CMD_is_MotorSelection) //Motor selection
      {
      case 0:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //Forward rotation
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //Reversal
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,/*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,/*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  switch (is_CarDirection)
  {
  case 1: /*Sport mode, left front*/
    ApplicationFunctionSet_ConquerorCarMotionControl(Left, is_CarSpeed);
    break;
  case 2: /*Sport mode, right front*/
    ApplicationFunctionSet_ConquerorCarMotionControl(Right, is_CarSpeed);
    break;
  case 3: /*Movement mode: Forward*/
    ApplicationFunctionSet_ConquerorCarMotionControl(Forward, is_CarSpeed);
    break;
  case 4: /*Sport mode Reverse*/
    ApplicationFunctionSet_ConquerorCarMotionControl(Backward, is_CarSpeed);
    break;
  default:
    break;
  }
}
/*
  N2: Command
  CMD mode: <Vehicle control> Receive and execute unidirectional vehicle control based on commands from the app.
  Time-limited.
*/
void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; //There is still time
  static boolean CarControl_return = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit) //Enter the vehicle with time-limited control mode
  {
    CarControl = true;
    if (is_Timer != 0) //#1 Set time to not be... hours (empty)
    {
      if ((millis() - Application_ConquerorCarxxx0.CMD_CarControl_Millis) > (is_Timer)) //Determine timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);

        Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //There is still time.
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(is_CarDirection, is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_ConquerorCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; //There is still time
  static boolean CarControl_return = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit) //Enter the vehicle with time-limited control mode
  {
    CarControl = true;
    if (CMD_is_CarTimer != 0) //#1 Set time to not be .. hours (empty)
    {
      if ((millis() - Application_ConquerorCarxxx0.CMD_CarControl_Millis) > (CMD_is_CarTimer)) //Check the timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);

        Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //There is still time.
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_ConquerorCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}
/*
  N3: Command
  CMD mode: <Vehicle control> Receive and execute unidirectional vehicle control based on commands from the app.   No time limit.
*/
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  static boolean CarControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit) //Enter the small car's unlimited time control mode
  {
    CarControl = true;
    CMD_CarControl(is_CarDirection, is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit) //Enter the small car's unlimited time control mode
  {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}

/*
  N4: Instruction
  CMD mode: Motion mode <motor control>
  Receive and execute control commands from the APP end to control the speed of the left and right motors.
*/
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R)
{
  static boolean MotorControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (is_Speed_L == 0 && is_Speed_R == 0)
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_Speed_L,/*direction_B*/ direction_just, /*speed_B*/ is_Speed_R,/*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(void)
{
  static boolean MotorControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0)
    {
      ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ CMD_is_MotorSpeed_L,/*direction_B*/ direction_just, /*speed_B*/ CMD_is_MotorSpeed_R,/*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}

/*
  N5: Instruction
  CMD mode: <Servo control>
*/
void ApplicationFunctionSet::CMD_ServoControl_xxx0(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_ServoControl)
  {
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo*/ CMD_is_Servo, /*unsigned int Position_angle*/ CMD_is_Servo_angle);
    Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
  }
}
/*
  N7: Command
   CMD mode: <Lighting control>
   Time limit: Enter programming mode after the time expires
*/
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B,uint32_t is_LightingTimer)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; //There is still time
  static boolean LightingControl_return = false;

  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit) //Enter time-limited control mode for lighting
  {
    LightingControl = true;
    if (is_LightingTimer != 0) //#1 Set time to not be .. hours (empty)
    {
      if ((millis() - Application_ConquerorCarxxx0.CMD_LightingControl_Millis) > (is_LightingTimer)) //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; //There is still time.
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_ConquerorCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; //There is still time
  static boolean LightingControl_return = false;

  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit) //Enter time-limited control mode for lighting
  {
    LightingControl = true;
    if (CMD_is_LightingTimer != 0) //#1 Set time to not be... hours (empty)
    {
      if ((millis() - Application_ConquerorCarxxx0.CMD_LightingControl_Millis) > (CMD_is_LightingTimer)) //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; //There is still time.
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_ConquerorCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
/*
  N8: Command
   CMD mode: <Lighting control>
   No time limit
*/
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  static boolean LightingControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit) //Enter lighting mode with no time limit
  {
    LightingControl = true;
    CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit) //Enter lighting mode with no time limit
  {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}

/*
  N100/N110: Command
  CMD mode: Clear all functions
*/
void ApplicationFunctionSet::CMD_ClearAllFunctions_xxx0(void)
{
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Standby_mode)  //Clear all functions: Enter idle mode    N100: Command
  {
    ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_ConquerorCarxxx0.Motion_Control = stop_it;
    Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
  }
  if (Application_ConquerorCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode) //Clear all functions: Enter programming mode     N110: Instruction
  {
    ApplicationFunctionSet_ConquerorCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_ConquerorCarxxx0.Motion_Control = stop_it;
    Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

/*
  N21: Instruction
  CMD mode: Ultrasonic module processing Receives and responds to control commands from the app end   Feedback ultrasonic status and data
  Input:
*/
void ApplicationFunctionSet::CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get)
{
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(const_cast<uint16_t*>(&UltrasoundData_cm) /*out*/); //Ultrasonic data
  UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  if (1 == is_get) //Ultrasonic  is_get Start     true: Obstacle present / false: No obstacle present
  {
    if (true == UltrasoundDetectionStatus)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  }
  else if (2 == is_get) //Ultrasonic is_get data
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}
/*
  N22: Instruction
  CMD mode: Tracking module receives and responds to control commands from the app, providing feedback on tracking status and data.
  Input:
*/
void ApplicationFunctionSet::CMD_TraceModuleStatus_xxx0(uint8_t is_get)
{
  char toString[10];
  if (0 == is_get) /*Get the left side of the trace status*/
  {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  else if (1 == is_get) /*Tracking status acquisition intermediate*/
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  else if (2 == is_get) /*Get the right side of the trace status*/
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  Application_ConquerorCarxxx0.Functional_Mode = CMD_Programming_mode; /*Enter programming mode prompt <Waiting for the next set of control commands>*/
}

/* 
 * End:CMD
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/*Key commands*/

void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;
    switch (get_keyValue)
    {
    case /* constant-expression */ 1:
      Application_ConquerorCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case /* constant-expression */ 2:
      Application_ConquerorCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case /* constant-expression */ 3:
      Application_ConquerorCarxxx0.Functional_Mode = Follow_mode;
      break;
    case /* constant-expression */ 4:
      Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
      break;
    default:

      break;
    }
  }
}

/*Infrared remote control*/
void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void)
{
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/))
  {
    IRrecv_en = true;
    //Serial.println(IRrecv_button);
  }
  if (true == IRrecv_en)
  {
    switch (IRrecv_button)
    {
    case /* constant-expression */ 1:

      Application_ConquerorCarxxx0.Motion_Control = Forward;
      break;
    case /* constant-expression */ 2:

      Application_ConquerorCarxxx0.Motion_Control = Backward;
      break;
    case /* constant-expression */ 3:

      Application_ConquerorCarxxx0.Motion_Control = Left;
      break;
    case /* constant-expression */ 4:

      Application_ConquerorCarxxx0.Motion_Control = Right;
      break;
    case /* constant-expression */ 5:

      //Application_ConquerorCarxxx0.Motion_Control = stop_it;
      Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
      break;

    case /* constant-expression */ 6:
 Application_ConquerorCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case /* constant-expression */ 7:
 Application_ConquerorCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case /* constant-expression */ 8:
 Application_ConquerorCarxxx0.Functional_Mode = Follow_mode;
      break;
    case /* constant-expression */ 9:
 if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode) //Adjust the response of the sensitive data segment of the tracking module
      {
        TrackingDetection_S += 10;
        if (TrackingDetection_S > 600)
        {
          TrackingDetection_S = 600;
        }
        else if (TrackingDetection_S < 30)
        {
          TrackingDetection_S = 30;
        }
      }

      break;
    case /* constant-expression */ 10:
 if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S = 250;
      }
      break;
    case /* constant-expression */ 11:
 if (Application_ConquerorCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S -= 10;
        if (TrackingDetection_S > 600)
        {
          TrackingDetection_S = 600;
        }
        else if (TrackingDetection_S < 30)
        {
          TrackingDetection_S = 30;
        }
      }
      break;

    default:
      Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
      break;
    }
    /*Direction control section implements time constraint control*/
    if (IRrecv_button < 5)
    {
      Application_ConquerorCarxxx0.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300)
      {
        IRrecv_en = false;
        Application_ConquerorCarxxx0.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    }
    else
    {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}






































/*Serial port data parsing*/ // wichtig
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  static String SerialPortData = "";
  char c = 0;
  if (Serial.available() > 0)
  {
    while (c != '}' && Serial.available() > 0)
    {
      // while (Serial.available() == 0)//强行等待一帧数据完成接收
      //   ;
      c = Serial.read();
      SerialPortData += (char)c;
    }
  }
  if (c == '}') //Data frame tail check
  {
#if _Test_print
    Serial.println(SerialPortData);
#endif

    // if (true == SerialPortData.equals("{f}") || true == SerialPortData.equals("{b}") || true == SerialPortData.equals("{l}") || true == SerialPortData.equals("{r}"))
    // {
    //   Serial.print(SerialPortData);
    //   SerialPortData = "";
    //   return;
    // }
    // if (true == SerialPortData.equals("{Factory}") || true == SerialPortData.equals("{WA_NO}") || true == SerialPortData.equals("{WA_OK}")) //避让测试架
    // {
    //   SerialPortData = "";
    //   return;
    // }
    StaticJsonDocument<200> doc;                                       //声明一个JsonDocument对象
    DeserializationError error = deserializeJson(doc, SerialPortData); //反序列化JSON数据
    SerialPortData = "";
    if (error)
    {
      //Serial.println("error:deserializeJson");
      return;
    }
    else if (!error) //检查反序列化是否成功
    {
      int control_mode_N = doc["N"];
      char *temp = doc["H"];
      CommandSerialNumber = temp; //获取新命令的序号

      /*以下代码块请结合通讯协议V.docx 查看*/
      switch (control_mode_N)
      {
      case 1: /*<命令：N 1> 电机控制模式 */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_MotorControl;
        CMD_is_MotorSelection = doc["D1"];
        CMD_is_MotorSpeed = doc["D2"];
        CMD_is_MotorDirection = doc["D3"];

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 2:                                                                    /*<命令：N 2> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_CarControl_TimeLimit; /*小车方向控制：有时间限定模式*/
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
        CMD_is_CarTimer = doc["T"];
        Application_ConquerorCarxxx0.CMD_CarControl_Millis = millis();
#if _is_print
        //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 3:                                                                      /*<命令：N 3> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_CarControl_NoTimeLimit; /*小车方向控制：无时间限定模式*/
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 4:                                                                  /*<命令：N 4> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_MotorControl_Speed; /*电机控制:控制转速模式*/
        CMD_is_MotorSpeed_L = doc["D1"];
        CMD_is_MotorSpeed_R = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 5:                                                            /*<命令：N 5> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_ServoControl; /*编程控制舵机*/
        CMD_is_Servo = doc["D1"];
        CMD_is_Servo_angle = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 7:                                                                         /*<命令：N 7> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_LightingControl_TimeLimit; /*灯光控制:有时间限定模式*/

        CMD_is_LightingSequence = doc["D1"]; //Lighting (Left, front, right, back and center)
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
        CMD_is_LightingTimer = doc["T"];
        Application_ConquerorCarxxx0.CMD_LightingControl_Millis = millis();
#if _is_print
        //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 8:                                                                           /*<命令：N 8> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_LightingControl_NoTimeLimit; /*灯光控制:无时间限定模式*/

        CMD_is_LightingSequence = doc["D1"]; //Lighting (Left, front, right, back and center)
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 21: /*<命令：N 21>：超声波模块:测距 */
        CMD_UltrasoundModuleStatus_xxx0(doc["D1"]);
#if _is_print
        //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 22: /*<命令：N 22>：红外模块：寻迹 */
        CMD_TraceModuleStatus_xxx0(doc["D1"]);
#if _is_print
        //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 23: /*<命令：N 23>：是否离开地面 */
        if (true == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_false}");
#endif
        }
        else if (false == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_true}");
#endif
        }
        break;

      case 110:                                                                                /*<命令：N 110> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Programming_mode; /*清除功能:进入编程模式*/

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 100:                                                                            /*<命令：N 100> */
        Application_ConquerorCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode; /*清除功能：进入空闲模式*/
#if _is_print
        Serial.print("{ok}");
        //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 101: /*<命令：N 101> :遥控切换命令*/
        if (1 == doc["D1"])
        {
          Application_ConquerorCarxxx0.Functional_Mode = TraceBased_mode;
        }
        else if (2 == doc["D1"])
        {
          Application_ConquerorCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
        }
        else if (3 == doc["D1"])
        {
          Application_ConquerorCarxxx0.Functional_Mode = Follow_mode;
        }

#if _is_print
        // Serial.print('{' + CommandSerialNumber + "_ok}");
        Serial.print("{ok}");
#endif
        break;

      case 105: /*<命令：N 105> :FastLED亮度调节控制命令*/
        if (1 == doc["D1"] && (CMD_is_FastLED_setBrightness < 250))
        {
          CMD_is_FastLED_setBrightness += 5;
        }
        else if (2 == doc["D1"] && (CMD_is_FastLED_setBrightness > 0))
        {
          CMD_is_FastLED_setBrightness -= 5;
        }
        FastLED.setBrightness(CMD_is_FastLED_setBrightness);

#if _Test_print
        //Serial.print('{' + CommandSerialNumber + "_ok}");
        Serial.print("{ok}");
#endif
        break;

      case 106: /*<命令：N 106> :/*摇杆控制舵机*/
      {
        uint8_t temp_Set_Servo = doc["D1"];
        if (temp_Set_Servo > 5 || temp_Set_Servo < 1)
          return;
        ApplicationFunctionSet_Servo(temp_Set_Servo);
      }

#if _is_print
      //Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
      break;
      case 102: /*<命令：N 102> :摇杆控制命令*/
        Application_ConquerorCarxxx0.Functional_Mode = Rocker_mode;
        Rocker_temp = doc["D1"];
        switch (Rocker_temp)
        {
        case 1:
          Application_ConquerorCarxxx0.Motion_Control = Forward;
          break;
        case 2:
          Application_ConquerorCarxxx0.Motion_Control = Backward;
          break;
        case 3:
          Application_ConquerorCarxxx0.Motion_Control = Left;
          break;
        case 4:
          Application_ConquerorCarxxx0.Motion_Control = Right;
          break;
        case 5:
          Application_ConquerorCarxxx0.Motion_Control = LeftForward;
          break;
        case 6:
          Application_ConquerorCarxxx0.Motion_Control = LeftBackward;
          break;
        case 7:
          Application_ConquerorCarxxx0.Motion_Control = RightForward;
          break;
        case 8:
          Application_ConquerorCarxxx0.Motion_Control = RightBackward;
          break;
        case 9:
          Application_ConquerorCarxxx0.Motion_Control = stop_it;
          break;
        default:
          Application_ConquerorCarxxx0.Motion_Control = stop_it;
          break;
        }
#if _is_print
        // Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      default:
        break;
      }
    }
  }
}

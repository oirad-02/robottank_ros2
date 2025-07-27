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

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

DeviceDriverSet_Servo AppServo;

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);
  AppServo.DeviceDriverSet_Servo_Init(90);
}


/*舵机控制*/
void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo; //防止被优化

  switch (temp_Set_Servo)
  {
  case 1 ... 2:
  {
    if (1 == temp_Set_Servo)
    {
      y_angle -= 1;
    }
    else if (2 == temp_Set_Servo)
    {
      y_angle += 1;
    }
    if (y_angle <= 5) //下限控制
    {
      y_angle = 5;
    }
    if (y_angle >= 11) //上下限控制
    {
      y_angle = 11;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == temp_Set_Servo)
    {
      z_angle += 1;
    }
    else if (4 == temp_Set_Servo)
    {
      z_angle -= 1;
    }

    if (z_angle <= 1) //下限控制
    {
      z_angle = 1;
    }
    if (z_angle >= 17) //上下限控制
    {
      z_angle = 17;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ 9);
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
    break;
  default:
    break;
  }
}

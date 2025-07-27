/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 16:36:20
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include "DeviceDriverSet_xxx0.h"

/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);

  delay(3000);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);
  delay(3000);
}
#endif

/*
 Motor_control：AB / 方向、速度
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                                          boolean direction_B, uint8_t speed_B, //B组电机参数
                                                          boolean controlED                     //AB使能允许 true
                                                          )                                     //电机控制
{
  // if (speed_A > speed_Max) //最大输出速度量
  // {
  //   speed_A = speed_Max;
  // }
  // if (speed_B > speed_Max) //最大输出速度量
  // {
  //   speed_B = speed_Max;
  // }

  if (controlED == control_enable) //使能允许？
  {
    { //A...Right

      switch (direction_A) //方向控制
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);

        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        break;
      }
    }
  }
  else
  {
    return;
  }
}

/*ULTRASONIC*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void)
{
  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);
}
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  *ULTRASONIC_Get = tempda_x;
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}



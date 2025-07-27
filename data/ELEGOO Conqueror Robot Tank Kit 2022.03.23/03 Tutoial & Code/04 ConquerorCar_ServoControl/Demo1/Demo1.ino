/*
   @Author: ELEGOO
   @Date: 2019-10-22 11:59:09
   @LastEditTime: 2020-06-28 14:55:26
   @LastEditors: Changhua
   @Description: conqueror robot tank
   @FilePath:
*/
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  for (int i = 1; i <= 5; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      Application_FunctionSet.ApplicationFunctionSet_Servo(i);
    }
  }
}

void loop()
{


}

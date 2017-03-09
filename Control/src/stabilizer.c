/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include "stabilizer.h"

static bool isInit;

// State variables for the stabilizer
 setpoint_t setpoint;
 sensorData_t sensorData;
 state_t state;
 control_t control;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
  
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE , NULL, 
              STABILIZER_TASK_PRI , NULL);
  
  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz. It is the responsability or the different
 * functions to run slower by skipping call (ie. returning without modifying
 * the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    sensorsAcquire(&sensorData, tick);

    stateEstimator(&state, &sensorData, tick);
    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &sensorData, &state, &setpoint, tick);
    powerDistribution(&control);

    tick++;
  }
}

//static void stabilizerTask(void* param)
//{
//  uint32_t lastWakeTime;
//
//  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_STABILIZER_ID_NBR);
//
//  //Wait for the system to be fully started to start stabilization loop
//  systemWaitStart();
//
//  lastWakeTime = xTaskGetTickCount();
//
//  for(;;)
//  {
//		
//    vTaskDelayUntil((portTickType *)&lastWakeTime,pdMS_TO_TICKS(2));//// 500Hz 
//
//    // Magnetometer not yet used more then for logging.
//	//imu9Read(&gyro, &acc, &mag);
//    imu6Read(&gyro,&acc);
//    if (imu6IsCalibrated())//判断传感器是否校准完成
//    {
//			//获取遥控器的期望欧拉角
//      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
//			//获取数据类型
//      commanderGetRPYType(&rollType, &pitchType, &yawType);
//      // 250HZ
//      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
//      {
//		//四元素更新
//        MahonyAHRSupdateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
//		//计算实际的欧拉角
//        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
//				
//		//返回在没有重力加速度下的垂直加速度
//        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
//				
//        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z) ;
//        // Estimate speed from acc (drifts)
//        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;
//
//        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
//                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
//                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
//        attitudeCounter = 0;				
//      }
//			
//      if(cnt++>100)
//      {	
//          Stm32QdcptLedToggle(LEDR);
//          cnt = 0;
//      }
//
//      // 100HZ 如果有气压计
//      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))//100hz进入一次
//      {
//        stabilizerAltHoldUpdate();
//        altHoldCounter = 0;
//      }
//
//      if (rollType == RATE)
//      {
//        rollRateDesired = eulerRollDesired;
//      }
//      if (pitchType == RATE)
//      {
//        pitchRateDesired = eulerPitchDesired;
//      }
//      if (yawType == RATE)
//      {
//        yawRateDesired = -eulerYawDesired;
//      }
//
//      // TODO: Investigate possibility to subtract gyro drift.
//      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
//                               rollRateDesired, pitchRateDesired, yawRateDesired);
//
//      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);
//
//      if (!altHold || !imuHasBarometer())//如果没有使用气压计定高
//      {
//        // Use thrust from controller if not in altitude hold mode
//        commanderGetThrust(&actuatorThrust);
////				actuatorThrust += accWZ*10000;
//      }
//      else
//      {
//        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
//        commanderWatchdog();
//      }
//			
//			BatVal=pmGetBatteryVoltage();
////			actuatorThrust = 0;
//			if(systemCanFly() &&eulerRollActual<90 &&eulerRollActual>-90)//
//			{
//				if (actuatorThrust > 0)
//				{
//						#if defined(TUNE_ROLL)
//					distributePower(actuatorThrust, actuatorRoll, 0, 0);
//						#elif defined(TUNE_PITCH)
//					distributePower(actuatorThrust, 0, actuatorPitch, 0);
//						#elif defined(TUNE_YAW)
//					distributePower(actuatorThrust, 0, 0, -actuatorYaw);
//						#else
//					distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
//						#endif
//				}
//				else
//				{
//					distributePower(0, 0, 0, 0);
//					controllerResetAllPID();
//				}
//			}
//			else
//			{
//					distributePower(0, 0, 0, 0);
//					controllerResetAllPID();
//			}
//    }
//  }
//}
//
//void StationLink(void)
//{
//  FloatToByte(eulerRollActual ,&EulerData.data[0]);
//  FloatToByte(eulerPitchActual,&EulerData.data[4]);	
//  FloatToByte(eulerYawActual  ,&EulerData.data[8]);	
//  FloatToByte(altHoldTarget,&EulerData.data[12]);//pmGetBatteryVoltage()
//  FloatToByte(actuatorThrust,&EulerData.data[16]);//actuatorThrust
//  FloatToByte(asl,&EulerData.data[20]);
//  FloatToByte(BatVal,&EulerData.data[24]);//
//  EulerData.size=28;
//  EulerData.header=0x80;
//  crtpSendPacketBlock(&EulerData);
//  //radioSendPacket();
//}
//
//void GetBaseThrust(void)
//{
//		if(BatVal>3.6)
//	{
//		altHoldBaseThrust= 36000;
//		altHoldMinThrust = 32000;	
//	}
//	else if(BatVal>3.5)
//	{
//		altHoldBaseThrust= 38000;
//		altHoldMinThrust = 34000;
//	}
//	else if(BatVal>3.4)
//	{
//		altHoldBaseThrust= 40000;
//		altHoldMinThrust = 36000;
//	}
//	else if(BatVal>3.3)
//	{
//		altHoldBaseThrust= 42000;
//		altHoldMinThrust = 38000;
//	}
//	else
//	{
//		altHoldBaseThrust= 48000;
//		altHoldMinThrust = 46000;
//	}
//}
//static void stabilizerAltHoldUpdate(void)
//{
//  // Cache last integral term for reuse after pid init
//  const float pre_integral = altHoldPID.integ;
//  // Get altitude hold commands from pilot
//	
//  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);
//	//获取目标高度值
//	//commanderGetHight(&HightDesired);
//  // Get barometer height estimates
//  //TODO do the smoothing within getData
//  ms5611GetData(&pressure, &temperature, &aslRaw);
//	//set the base thrust
//	
//	GetBaseThrust();
//	
//  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
//  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);
//
//  // Estimate vertical speed based on successive barometer readings. This is ugly :)
//  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);
//
//  // Estimate vertical speed based on Acc - fused with baro to reduce drift
//  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
////  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
//  vSpeedAcc = vSpeed;
//
//  // Reset Integral gain of PID controller if being charged
//// if (!pmIsDischarging())
////  {
////    altHoldPID.integ = 0.0;
////  }
//
//
//  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
//  if (setAltHold)//
//  {
//    // Set to current altitude
//    altHoldTarget = asl;
//
//    // Reset PID controller
//    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
//            ALTHOLD_UPDATE_DT);
//    // TODO set low and high limits depending on voltage
//    // TODO for now just use previous I value and manually set limits for whole voltage range
//    //                    pidSetIntegralLimit(&altHoldPID, 12345);
//    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /
////		Stm32QdcptLedToggle(LEDL);
//    altHoldPID.integ = pre_integral;
//
//    // Reset altHoldPID
//    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
//  }
//	
//  // In altitude hold mode
//  if (altHold)//altHold
//  {
////		Stm32QdcptLedToggle(LEDR);
//    // Update target altitude from joy controller input
//    altHoldTarget += altHoldChange / altHoldChange_SENS;
////		altHoldErr =asl - altHoldTarget;
//    pidSetDesired(&altHoldPID, altHoldTarget);
//
//    // Compute error (current - target), limit the error
//    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
//                           -altHoldErrMax, altHoldErrMax);
//		
//    pidSetError(&altHoldPID, -altHoldErr);
//
//    // Get control from PID controller, dont update the error (done above)
//    // Smooth it and include barometer vspeed
//    // TODO same as smoothing the error??+accWZ*10
//    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
//                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));
//
//    // compute new thrust
//    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
//                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));
//		
//    // i part should compensate for voltage drop
//
//  }
//  else
//  {
//    altHoldTarget = 0.0;
//    altHoldErr = 0.0;
//    altHoldPIDVal = 0.0;
//  }
//}
//
//static void distributePower(const uint16_t thrust, const int16_t roll,
//                            const int16_t pitch, const int16_t yaw)
//{
//#ifdef QUAD_FORMATION_X
//  int16_t intRoll, intPitch;
//  
//  intRoll  = roll  >> 1;
//  intPitch = pitch >> 1;
//  motorPowerM1 = limitThrust(thrust + roll + pitch - yaw);
//  motorPowerM2 = limitThrust(thrust + roll - pitch + yaw);
//  motorPowerM3 = limitThrust(thrust - roll - pitch - yaw);
//  motorPowerM4 = limitThrust(thrust - roll + pitch + yaw);
////  motorPowerM1 = limitThrust(thrust + roll + pitch + yaw);
////  motorPowerM2 = limitThrust(thrust + roll - pitch - yaw);
////  motorPowerM3 = limitThrust(thrust - roll - pitch + yaw);
////  motorPowerM4 = limitThrust(thrust - roll + pitch - yaw);
//#else // QUAD_FORMATION_NORMAL
//  motorPowerM1 = limitThrust(thrust + pitch + yaw);
//  motorPowerM2 = limitThrust(thrust - roll - yaw);
//  motorPowerM3 = limitThrust(thrust - pitch + yaw);
//  motorPowerM4 = limitThrust(thrust + roll - yaw);
//#endif
//
//  motorsSetRatio(MOTOR_M1, motorPowerM1);
//  motorsSetRatio(MOTOR_M2, motorPowerM2);
//  motorsSetRatio(MOTOR_M3, motorPowerM3);
//  motorsSetRatio(MOTOR_M4, motorPowerM4);
//}
//
//static uint16_t limitThrust(int32_t value)
//{
//  if(value > UINT16_MAX)
//  {
//    value = UINT16_MAX;
//  }
//  else if(value < 0)
//  {
//    value = 0;
//  }
//
//  return (uint16_t)value;
//}
//
//// Constrain value between min and max
//static float constrain(float value, const float minVal, const float maxVal)
//{
//  return min(maxVal, max(minVal,value));
//}
//
//// Deadzone
//static float deadband(float value, const float threshold)
//{
//  if (fabs(value) < threshold)
//  {
//    value = 0;
//  }
//  else if (value > 0)
//  {
//    value -= threshold;
//  }
//  else if (value < 0)
//  {
//    value += threshold;
//  }
//  return value;
//}

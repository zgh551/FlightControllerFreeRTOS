#include "TimerTask.h"

float IMU_Dat[8],MAG_Dat[4],CorrectMAG_Dat[4];
float m_YAW,m_PITCH,m_ROLL;
int16_t IMU_Dat_Init[7];
uint16_t time_100ms=0,time_5ms;
uint16_t cnt=0;
FRESULT Task_res;

void Tim1Task_isr(void)
{
  if(TIM_GetFlagStatus(TIM1,TIM_FLAG_Update) != RESET )
  {
    time_100ms = (time_100ms + 1)%20;    
    //5ms Task
    ICM20601_getData(IMU_Dat);
    HMC5983GetFloatData(MAG_Dat);
//  MagCorrect(MAG_Dat,CorrectMAG_Dat);
    MagCorrectOffset(MAG_Dat,CorrectMAG_Dat);
//    printf("%f %f %f\r\n",CorrectMAG_Dat[0],CorrectMAG_Dat[1],CorrectMAG_Dat[2]);
    MahonyAHRSupdateIMU(IMU_Dat[4],-IMU_Dat[5],-IMU_Dat[6],
                        IMU_Dat[1],-IMU_Dat[2],-IMU_Dat[3]);
    MahonyAHRSupdate(IMU_Dat[4],-IMU_Dat[5],-IMU_Dat[6],
                     IMU_Dat[1],-IMU_Dat[2],-IMU_Dat[3],
                     -CorrectMAG_Dat[1],-CorrectMAG_Dat[0],-CorrectMAG_Dat[2]);
    GetTheEuler(&m_ROLL,&m_PITCH,&m_YAW);
    printf("%3f %3f %3f\r\n",m_YAW,m_ROLL,m_PITCH);
    //100ms Task
    if(time_100ms == 0)
    {
      STM_LED_Toggle(LEDL);
    }
    TIM_ClearFlag(TIM1,TIM_FLAG_Update);
  }
}
//      AHRSupdate(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4]);    
//      IMU_Gyro_Updata_Mag_Correct(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                                  MAG_Dat[0],MAG_Dat[1],MAG_Dat[2]); 
//      MadgwickAHRSupdateIMU(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                            IMU_Dat[3],IMU_Dat[2],-IMU_Dat[1]);
//      MahonyAHRSupdateIMU(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                          IMU_Dat[3],IMU_Dat[2],-IMU_Dat[1]);  
      
//      MahonyAHRSupdate( IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                        IMU_Dat[3],IMU_Dat[2],-IMU_Dat[1],
//                        MAG_Dat[2],MAG_Dat[0],MAG_Dat[1]); 
      
//      ZghAHRSupdate(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                    MAG_Dat[2],MAG_Dat[0],MAG_Dat[1]);
//      Madgwick_ZGH_AHRSupdate(IMU_Dat[6],IMU_Dat[5],-IMU_Dat[4],
//                              MAG_Dat[2],MAG_Dat[0], MAG_Dat[1]);
      
      
//      GetTheEuler(&m_YAW,&m_ROLL,&m_PITCH);
//      printf("%3f %3f %3f ",m_YAW,m_ROLL,m_PITCH);
//      IMU_Gyro_Updata_Mag_Correct(m_YAW,m_PITCH,MAG_Dat[2],MAG_Dat[1],MAG_Dat[0]);
      
      
//      printf("%f %f %f\r\n",IMU_Dat[4],IMU_Dat[5],IMU_Dat[6]);
//      MAG_Dat[0] = m_PITCH;
//      MAG_Dat[1] = m_ROLL;
//      MAG_Dat[2] = m_YAW;
//      HMC5983SendData(MAG_Dat);
////      HMC5983SendData(&IMU_Dat[4]);
//      printf("%f %f %f %f %f %f %f %f %f %f \r\n",
//          IMU_Dat[0],
//          IMU_Dat[1],IMU_Dat[2],IMU_Dat[3],
//          IMU_Dat[4],IMU_Dat[5],IMU_Dat[6],
//          MAG_Dat[0],MAG_Dat[1],MAG_Dat[2]);
//      printf("%3f %3f %3f\r\n",m_YAW,m_ROLL,m_PITCH);
      //x y z
//      printf("%f %f %f\r\n",MAG_Dat[2],MAG_Dat[1],MAG_Dat[0]);
//      printf("AX=%f AY%f %f\r\n",IMU_Dat[1],IMU_Dat[2]);
//    }
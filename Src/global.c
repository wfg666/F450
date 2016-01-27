#include "global.h"

uint8_t USART2RxBuffer[128];
uint8_t USART3RxBuffer[128];
uint16_t rcChannels[18];
float PWMOutput[4];


__IO struct AHRSDataStruct AHRSData;

__IO float pitchAngleP=0.00035,rollAngleP=0.00035,yawAngleP=0.00045;
__IO float pitchAngleI=0,rollAngleI=0,yawAngleI=0;
__IO float pitchAngleD=0.00002,rollAngleD=0.00002,yawAngleD=0.00003;

__IO float pitchAngleSP,rollAngleSP,yawAngleSP=0;

__IO float throttle;

__IO float pwm1,pwm2,pwm3,pwm4;

__IO float pitchAngleErr,rollAngleErr,yawAngleErr;
__IO float pitchAngleLastErr,rollAngleLastErr,yawAngleLastErr;
__IO float pitchAngleErrInte,rollAngleErrInte,yawAngleErrInte;
__IO float pitchAngleAdjust,rollAngleAdjust,yawAngleAdjust;



__IO int32_t sysClock=0; //timer6 0.1ms   max:1 000 000 000

__IO uint8_t AHRSReceived=0;
__IO uint8_t rcReceived=0;
__IO uint8_t shouldUpdateControl=0;

__IO int32_t lastUpdateControlTime=0;

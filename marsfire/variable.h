/* variables.h
 *  Header file that contains all the variable declarations for the 
 *  MARS CONTROL program.
 *  
 *  Sivakumar Balasubramanian.
 */

#include "Arduino.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include "SoftwareSerial.h"
#include "HX711_ADC.h"
#include <Bounce2.h>






float time_ellapsed = 0;


IntervalTimer readStream;
IntervalTimer writeStream;
IntervalTimer IMUupdate;

SoftwareSerial bt(0,1);

unsigned long timer = 0;

int calib_button_pin = 33;
float calibButtonState;

// motor1 initialization
int PWMpin_theta1 = 37;
int enablepin_theta1 = 38;
int directionpin_theta1 = 39;

//ENCODER
#define ENC1A           2//4
#define ENC1B           3//5
#define ENC2A           4//11
#define ENC2B           5//10
#define ENC3A           6//12
#define ENC3B           7//13
#define ENC4A           8//36
#define ENC4B           9//35
//#define ENC5A           //22
//#define ENC5B           //23

#define ENC1MAXCOUNT    4*4096*53
#define ENC1COUNT2DEG   0.25f*0.00166f
#define ENC2MAXCOUNT    4*1024
#define ENC2COUNT2DEG   0.25f*0.3515625f
#define ENC3MAXCOUNT    4*1024
#define ENC3COUNT2DEG   0.25f*0.3515625f
#define ENC4MAXCOUNT    4*1024
#define ENC4COUNT2DEG   0.25f*0.3515625f
//#define ENC5MAXCOUNT    4*1024
//#define ENC5COUNT2DEG   0.25f*0.351625f

// Variable to hold the current Mars button state.
volatile byte marsButton = 0;
Bounce button = Bounce();
#define IO_SWITCH           20

Encoder angle1(ENC1A, ENC1B);
long _enccount1;

Encoder angle2(ENC2A, ENC2B);
long _enccount2;

Encoder angle3(ENC3A, ENC3B);
long _enccount3;

Encoder angle4(ENC4A, ENC4B);
long _enccount4;
//Encoder angle5(ENC5A, ENC5B);


//Loadcell
#define LOADCELL1_DOUT_PIN 14//15
#define LOADCELL1_SCK_PIN 15//16
#define LOADCELL2_DOUT_PIN 16//17
#define LOADCELL2_SCK_PIN 17//18
#define calibration_factor 12866.6

//HX711 scale1;
HX711_ADC scale1(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
HX711_ADC scale2(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

float angle_received;
float Received_ang, payLoadSize, btStatus, btStatus1;

float des1, des2, des3, PCParam;
int stat = 0;
int handUse;
float controlSense;
float force1, force2, fAct, fDes, fActiminus1;
float errorLoadCell;

//Sensor data
float theta1, theta2, theta3, theta4, theta5;
float th1, th2, th3, th4;
float upperArm, foreArm;
float shx, shy, shz;
float W1, W2;

float support, endx, endy, endz;
float zvec1, zvec2, zvec3;
float elbx, elby, elbz;
float fAx, fAy, fAz;
float uAx, uAy, uAz;
float phi1, phi2, phi4, dot;
float distFromZAxis;

float sigmoid, meanx = 0.1, spreadx = 0.03, theta1x;
float sigmoidFlex, meanFlex = 140, spreadFlex = 7;
float tarm, tact, fdes, tweight, fDesiminus1;
float Kp_theta1, Kd_theta1;
float errorForce, errorForcei, controllaw_theta1, taud, motorcurrent_theta1, theta1d, erroriminus1_theta1;
float error_theta1, errord_theta1, theta1iminus1, PWM_theta1, PCParamiminus1;
float n;

int offsetCnt;
float theta1Enc, theta2Enc, theta3Enc, theta4Enc;
float IMUtheta1, IMUtheta2, IMUtheta3, IMUtheta4, offset1, offset2, offset3, offset4;
float ax1, ay1, az1, ax2, ay2, az2, ax3, ay3, az3;
float norm1, norm2, norm3;

// Serial Reader object
SerialReader serReader;
// Out data buffer
::OutDataBuffer4Float outPayload;

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
#include "Wire.h"
#include <MPU6050_light.h>

// Define the PIN numbers
#define CALIB_BUTTON          33
#define MARS_BUTTON           20
// int calib_button_pin = 33;
#define MOTOR_PWM             37
#define MOTOR_ENABLE          38
#define MOTOR_DIR             39
// int PWMpin_theta1 = 37;
// int enablepin_theta1 = 38;
// int directionpin_theta1 = 39;
// Robot Encoders
#define ENC1A                 2//4
#define ENC1B                 3//5
#define ENC2A                 4//11
#define ENC2B                 5//10
#define ENC3A                 6//12
#define ENC3B                 7//13
#define ENC4A                 8//36
#define ENC4B                 9//35
// Encoder parameters     
#define ENC1MAXCOUNT          4 * 4096 * 53
#define ENC1COUNT2DEG         0.25f * 0.00166f
#define ENC2MAXCOUNT          4 * 1024
#define ENC2COUNT2DEG         0.25f * 0.3515625f
#define ENC3MAXCOUNT          4 * 1024
#define ENC3COUNT2DEG         0.25f * 0.3515625f
#define ENC4MAXCOUNT          4 * 1024
#define ENC4COUNT2DEG         0.25f * 0.3515625f
// Load cells
#define LOADCELL1_DOUT_PIN    14//15
#define LOADCELL1_SCK_PIN     15//16
#define LOADCELL2_DOUT_PIN    16//17
#define LOADCELL2_SCK_PIN     17//18
#define LOACELL_CALIB_FACTOR  12866.6

// Limb type
#define NOLIMB                0x00
#define RIGHT                 0x01
#define LEFT                  0x02

// Control type
#define NONE                  0x00
#define POSITION              0x01
#define TORQUE                0x02
#define ARM_WEIGHT_SUPPORT    0x03

// Out data type
#define SENSORSTREAM          0x00
#define CONTROLPARAM          0x01
#define DIAGNOSTICS           0x02
#define VERSION               0x03

// In data type
#define GET_VERSION           0x00
#define RESET_PACKETNO        0x01
#define SET_LIMB              0x02
#define CALIBRATE             0x03
#define START_STREAM          0x04
#define STOP_STREAM           0x05
#define SET_CONTROL_TYPE      0x06
#define SET_CONTROL_TARGET    0x07
#define SET_DIAGNOSTICS       0x08
#define HEARTBEAT             0x80

// Control Law Related Definitions
#define INVALID_TARGET        999.0
#define INTEGRATOR_LIMIT      4.0
#define PWMRESOLN             12      // This has been changed from 8. Suggestions from Aravind.
#define MINPWM                410     // 10% of 4095
#define MAXPWM                3686    // 90% of 4095
#define MAXDELPWM             40      // Changed from 5

// Error types 
#define ANGSENSERR            0x0001
#define MCURRSENSERR          0x0002
#define NOHEARTBEAT           0x0004

// Kinematic calib status
#define NOCALIB               0x00
#define YESCALIB              0x01

// IMU offsets
#define IMU1OFFSET            0.00
#define IMU2OFFSET            0.00
#define IMU3OFFSET            0.00
#define IMU4OFFSET            0.00

// Calibration angle limits
#define CALIB_IMU_ANGLE_MIN   -20.0
#define CALIB_IMU_ANGLE_MAX   +20.0

// Human limb parameter ranges
#define MIN_UA_LENGTH         200     // millimeters
#define MAX_UA_LENGTH         400     // millimeters
#define MIN_FA_LENGTH         200     // millimeters
#define MAX_FA_LENGTH         500     // millimeters
#define MIN_UA_WEIGHT         1.0     // Kg
#define MAX_UA_WEIGHT         5.0     // Kg
#define MIN_FA_WEIGHT         1.0     // Kg
#define MAX_FA_WEIGHT         5.0     // Kg
#define MIN_SHLDR_X_POS       -150    // millimeters
#define MAX_SHLDR_X_POS       +150    // millimeters
#define MIN_SHLDR_Y_POS       -150    // millimeters
#define MAX_SHLDR_Y_POS       +150    // millimeters
#define MIN_SHLDR_Z_POS       100     // millimeters
#define MAX_SHLDR_Z_POS       500     // millimeters

// Heart beat related variable
#define MAX_HBEAT_INTERVAL    2.0 // Seconds

// Radians to degree conversion
#define RAD2DEG(x)            180.0 * x / 3.141592

// Version and device ID.
const char* fwVersion = "25.07";
const char* deviceId  = "MARS-HOMER";
const char* compileDate = __DATE__ " " __TIME__;

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// Program status
byte streamType = SENSORSTREAM;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
uint16union_t deviceError;

// Packet Counter.
uint16union_t packetNumber;

// run time
unsigned long startTime;
ulongunion_t runTime;

// Current limb
byte currLimb;

// Control related variables.
float target;

// Kinematics related variables.
Encoder angle1(ENC1A, ENC1B);
long _enccount1;
Encoder angle2(ENC2A, ENC2B);
long _enccount2;
Encoder angle3(ENC3A, ENC3B);
long _enccount3;
Encoder angle4(ENC4A, ENC4B);
long _enccount4;
float limbScale1, limbScale2, limbScale3, limbScale4; 
float theta1, theta2, theta3, theta4;

// Calibration related variables.
float theta1Offset, theta2Offset, theta3Offset, theta4Offset;

// IMU angles and their codes.
MPU6050 mpu(Wire);
MPU6050 mpu2(Wire1);
MPU6050 mpu3(Wire1);
float imuTheta1, imuTheta2, imuTheta3, imuTheta4;
int8_t imu1Byte, imu2Byte, imu3Byte, imu4Byte;

// Human limb parameters and their codes.
float uaLength, faLength;
uint8_t uaLByte, faLByte;
float shPosZ;
uint8_t shZByte;
float uaWeight, faWeight;
uint8_t uaWByte, faWByte;


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

float time_ellapsed = 0;
IntervalTimer readStream;
// IntervalTimer writeStream;
// IntervalTimer IMUupdate;

SoftwareSerial bt(0,1);

unsigned long timer = 0;

float calibButtonState;

// motor1 initialization


//#define ENC5MAXCOUNT    4*1024
//#define ENC5COUNT2DEG   0.25f*0.351625f

// MARS nad Calibration buttons states.
volatile byte marsButton = 0;
volatile byte calibButton = 0;
volatile byte devButtons = 0;
Bounce marsBounce = Bounce();
Bounce calibBounce = Bounce();


//Encoder angle5(ENC5A, ENC5B);



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



float sigmoid, meanx = 0.1, spreadx = 0.03, theta1x;
float sigmoidFlex, meanFlex = 140, spreadFlex = 7;
float tarm, tact, fdes, tweight, fDesiminus1;
float Kp_theta1, Kd_theta1;
float errorForce, errorForcei, controllaw_theta1, taud, motorcurrent_theta1, theta1d, erroriminus1_theta1;
float error_theta1, errord_theta1, theta1iminus1, PWM_theta1, PCParamiminus1;
float n;

int offsetCnt;
float theta1Enc, theta2Enc, theta3Enc, theta4Enc;

// Serial Reader object
SerialReader serReader;
// Out data buffer
OutDataBuffer4Float outPayload;

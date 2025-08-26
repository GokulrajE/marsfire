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
#include "Wire.h"
#include <MPU6050_light.h>

// Define the PIN numbers
#define CALIB_BUTTON          33
#define MARS_BUTTON           20
// int calib_button_pin = 33;
#define MOTOR_PWM             37
#define MOTOR_ENABLE          38
#define MOTOR_DIR             39
#define MOTOR_T2I             3.35  // A / Nm
#define MOTOR_TORQ_CONST      3.7153  // Nm / A
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
#define ARM_REST_WEIGHT       1.46    // New

// Limb type
#define NOLIMB                0x00
#define RIGHT                 0x01
#define LEFT                  0x02

// Control type
#define NONE                  0x00
#define POSITION              0x01
#define TORQUE                0x02
#define AWS                   0x03    // Arm Weight Support

// Out data type
#define SENSORSTREAM          0x00
#define CONTROLPARAM          0x01
#define DIAGNOSTICS           0x02
#define HLIMKINPARAM          0x03
#define HLIMDYNPARAM          0x04
#define VERSION               0x05

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
#define SET_LIMB_KIN_PARAM    0x09
#define SET_LIMB_DYN_PARAM    0x0A
#define GET_LIMB_KIN_PARAM    0x0B
#define GET_LIMB_DYN_PARAM    0x0C
#define RESET_LIMB_KIN_PARAM  0x0D
#define RESET_LIMB_DYN_PARAM  0x0E
#define TRANSITION_CONTROL    0x0F
#define HEARTBEAT             0x80

// Control Law Related Definitions
#define INVALID_TARGET        999.0
#define INTEGRATOR_LIMIT      4.0
#define POS_ERROR_CAP         10.0    // Degrees
#define POS_ERROR_DIFF_CAP    2.0     // Degrees
#define PWMRESOLN             12      // This has been changed from 8. Suggestions from Aravind.
#define MINPWM                410     // 10% of 4095
#define MAXPWM                3686    // 90% of 4095
#define MAXDELPWM             40      // Changed from 5
#define MAX_CURRENT           10
#define POS_CTRL_DBAND        0
#define POSITION_TARGET_MIN   -120    // Degrees
#define POSITION_TARGET_MAX   20      // Degrees
#define TORQUE_CTRL_DBAND     0
#define TORQUE_TARGET_MIN     -20     // Nm
#define TORQUE_TARGET_MAX     20      // Nm
#define POSITION_RATE_LIMIT   5       // Degrees / sec
#define TORQUE_RATE_LIMIT     0.25    // Nm / sec
#define TORQTGT_SIG_MA_MID    0.15    // m       
#define TORQTGT_SIG_MA_SPRD   0.10    // m
#define TORQTGT_SIG_FLX_MID   -100    // Degrees       
#define TORQTGT_SIG_FLX_SPRD  5       // Degrees
#define AWS_TARGET_MIN        0.0     // Proportion of arm weight to support.
#define AWS_TARGET_MAX        1.0     // Proportion of arm weight to support.
#define AWS_TRANS_FACTOR      0.995
#define AWS_SCALE_FACTOR      1.0     // A fixed scale factor for arm weight support.
#define SAFETY_DAMP_VEL_TH    10.0    // deg / sec
#define SAFETY_DAMP_VALUE     10.0    // PWM / (deg / sec)

// Error types 
#define ANGSENSERR            0x0001
#define MCURRSENSERR          0x0002
#define NOHEARTBEAT           0x0004

// Safety timer thresholds
#define TARGET_SET_BACKOUT    2000     // millisec

// Kinematic calib status
#define NOCALIB               0x00
#define YESCALIB              0x01

// Recent command status
#define COMMAND_NONE          0x00
#define COMMAND_SUCCESS       0x01
#define COMMAND_FAIL          0x02

// Button bounce threshold
#define BOUNCE_THRESHOLD      5

// Limb parameter status
#define NOLIMBKINPARAM        0x00
#define YESLIMBKINPARAM       0x01
#define NOLIMBDYNPARAM        0x00
#define YESLIMBDYNPARAM       0x01

// IMU offsets
#define IMU1OFFSET            0.00
#define IMU2OFFSET            -9.3
#define IMU3OFFSET            -3.0
#define IMU4OFFSET            -4.75

// Calibration angle limits
#define CALIB_IMU_ANGLE_MIN   -20.0
#define CALIB_IMU_ANGLE_MAX   +20.0

// Control related constant
#define MARS_GRAV_COMP_ADJUST 1.5
#define MIN_TARGET_DUR        2.0     // Seconds

// MARS robot parameters.
#define L1                    0.475   // meters
#define L2                    0.291   // meters

// Human limb parameter ranges
#define MIN_UA_LENGTH         0.150   // meters
#define MAX_UA_LENGTH         0.400   // meters
#define MIN_FA_LENGTH         0.100   // meters
#define MAX_FA_LENGTH         0.300   // meters
#define MIN_UA_WEIGHT         -8.00   // Kg
#define MAX_UA_WEIGHT         -1.00   // Kg
#define MIN_FA_WEIGHT         -5.00   // Kg
#define MAX_FA_WEIGHT         +1.00   // Kg
#define MIN_SHLDR_X_POS       -0.15   // meters
#define MAX_SHLDR_X_POS       +0.15   // meters
#define MIN_SHLDR_Y_POS       -0.15   // meters
#define MAX_SHLDR_Y_POS       +0.15   // meters
#define MIN_SHLDR_Z_POS       0.100   // meters
#define MAX_SHLDR_Z_POS       0.500   // meters

// Heart beat related variable
#define MAX_HBEAT_INTERVAL    2.0 // Seconds

// Radians to degree conversion
#define RAD2DEG(x)            180.0 * x / 3.141592
#define DEG2RAD(x)            3.141592 * x / 180.0

// Sigmoid function
#define SIGMOID(x)            1 / (1 + exp(-5 * x))

// Version and device ID.
const char* fwVersion = "25.08";
const char* deviceId  = "MARS-HOMER";
const char* compileDate = __DATE__ " " __TIME__;

// MARS Gravity Compensation Parameters
const float marsGCParam[] = {
  -0.00423728,
  -1.18992423,
   3.74296361,
   1.19731609,
   1.99763005,
   0.24580473,
  -0.16474236,
   1.30690941
};

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// Program status
byte streamType = SENSORSTREAM;
byte cmdStatus = COMMAND_NONE;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
byte limbKinParam = NOLIMBKINPARAM;
byte limbDynParam = NOLIMBDYNPARAM;
uint16union_t deviceError;

// Packet Counter.
uint16union_t packetNumber;

// run time
unsigned long startTime;
ulongunion_t runTime;
unsigned long currMilliCount;
unsigned long prevMilliCount;
float delTime;

// Current limb
byte currLimb;

// MARS and Calibration buttons states.
int8_t marsBounceCount = 0;
byte marsButton = 0;
int8_t calibBounceCount = 0;
byte calibButton = 0;
byte devButtons = 0;

// Control related variables.
float target;
Buffer actual;
Buffer desired;
Buffer control;

// Joint kinematics related variables.
Encoder angle1(ENC1A, ENC1B);
long _enccount1;
Encoder angle2(ENC2A, ENC2B);
long _enccount2;
Encoder angle3(ENC3A, ENC3B);
long _enccount3;
Encoder angle4(ENC4A, ENC4B);
long _enccount4;
float limbAngleScale;
float limbControlScale; 
float theta1, theta2, theta3, theta4;
float theta1Prev, theta2Prev, theta3Prev, theta4Prev;
float theta1r, theta2r, theta3r, theta4r;
float omega1, omega2, omega3, omega4;

// Cosine and sine terms of the individual angles.
float cos1, cos2, cos3, cos4;
float sin1, sin2, sin3, sin4;

// Calibration related variables.
float theta1Offset, theta2Offset, theta3Offset, theta4Offset;

// IMU angles and their codes.
MPU6050 mpu(Wire);
MPU6050 mpu2(Wire1);
MPU6050 mpu3(Wire1);
float imuTheta1, imuTheta2, imuTheta3, imuTheta4;
int8_t imu1Byte, imu2Byte, imu3Byte, imu4Byte;

// Endpoint kinematics of the robot.
float xEp, yEp, zEp;
float momentArm;

// Endpoint force.
float epForce;
float torque, torquePrev;
float dTorque;

// Variables to handle transition between POSITION and AWS control.
float transitionTorque = 0;
float beta = 0;

// Human limb torque
float hLimbTorque;

// Human limb parameters and their codes.
float uaLength, faLength;
// Square of the limb lengths which will be used for inverse kinematics.
float uaL2, faL2, uafaL;
// uint8_t uaLByte, faLByte;
float shPosZ;
// uint8_t shZByte;
float uaWeight, faWeight;
// Human joint angles.
float phi1, phi2, phi3;

// Controller gains
float pcKp = 1.5;
float pcKd = 7.8;
float pcKi = 0;
float tcKp = 0.5;
float tcKd = 0.7;
float tcKi = 0.1;
// Control related buffers
float err;
float errdiff;
float errsum;
float marsGCTorque;
// Desired target related variables.
float strtPos, strtTime, initTime, tgtDur; 

// Temporary variable for parsing incoming data.
float tempArray[8];

// Safety Time Flags.
unsigned long targetSetTime;


// Old variables.
float th1, th2, th3, th4;
float upperArm, foreArm;
float shx, shy, shz;
float W1, W2;
float support, endx, endy, endz;
float zvec1, zvec2, zvec3;
float elbx, elby, elbz;
float fAx, fAy, fAz;
float uAx, uAy, uAz;
// float phi1, phi2, phi4, dot;
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

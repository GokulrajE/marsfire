
/* variables.h
 *  Header file that contains all the variable declarations for the 
 *  MARSFIRE program.
 *  
 *  Sivakumar Balasubramanian.
 */
#include <Bounce2.h>
#include <Encoder.h>

#include "RGBLed.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include "SoftwareSerial.h"
#include "HX711_ADC.h"

// Encoder pins
// Theta 1
#define ENC1A               5
#define ENC1B               4
// Theta 2
#define ENC2A               2
#define ENC2B               3
// Theta 3
#define ENC3A               8
#define ENC3B               9
// Theta 4
#define ENC4A               7
#define ENC4B               6


// Encoder constants
#define ENC1MAXCOUNT        4 * 4096 * 53
#define ENC1COUNT2DEG       0.25f * 0.00166f
#define ENC2MAXCOUNT        4 * 1024
#define ENC2COUNT2DEG       0.25f * 0.3515625f
#define ENC3MAXCOUNT        4 * 1024
#define ENC3COUNT2DEG       0.25f * 0.3515625f
#define ENC4MAXCOUNT        4 * 1024
#define ENC4COUNT2DEG       0.25f * 0.3515625f

// Loadcell pins
#define LC1_DOUT_PIN        14
#define LC1_SCK_PIN         15
#define LC2_DOUT_PIN        16
#define LC2_SCK_PIN         17
#define LC_CALIB_FACTOR     12866.6f

// MARS Buttons
#define IO_SWITCH           17

// Motor pins
#define PWM                 37
#define ENABLE              38
#define CW                  39

// Control type
#define NONE                0x00
#define POSITION            0x01
#define FORCE               0x02
#define WEIGHTSUPPORT       0x03

// Out data type
#define VERSION             0x00
#define SENSORSTREAM        0x01
#define DIAGNOSTICS         0x02
#define SUBJECTPARAM        0x03

// In data type
#define GET_VERSION         0x00
#define CALIBRATE           0x01
#define START_STREAM        0x02
#define STOP_STREAM         0x03
#define SET_DIAGNOSTICS     0x04
#define SET_CONTROL_TYPE    0x05
#define SET_CONTROL_TARGET  0x06
#define RESET_PACKETNO      0x07
#define HEARTBEAT           0x80

// Control Law Related Definitions
#define INVALID_TARGET      999.0
#define INTEGRATOR_LIMIT    4.0
#define PWMRESOLN           12      // This has been changed from 8. Suggestions from Aravind.
#define MINPWM              410     // 10% of 4095
#define MAXPWM              3686    // 90% of 4095
#define MAXDELPWM           40      // Changed from 5

// Error types 
#define NOHEARTBEAT         0x0001
#define ANGSENSERR          0x0002
#define MCURRSENSERR        0x0003
#define LCSESNSERR          0x0004

// Kinematic calib status
#define NOCALIB             0x00
#define YESCALIB            0x01

// Control related variables
#define POS_CTRL_DBAND      2

// Motor constants
#define MAX_CURRENT         8

// Heart beat related variable
#define MAX_HBEAT_INTERVAL  1.0 // Seconds

// Some useful function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Nonlinear PID controller functions.
#define linclip(x) ((x) < 0 ? 0 : ((x) > 1 ? 1 : x))

// Version and device ID.
const char* fwVersion = "25.01";
const char* deviceId  = "MARS250225";
const char* compileDate = __DATE__ " " __TIME__;

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// Encoder variables.
Encoder encTheta1(ENC1A, ENC1B);
Encoder encTheta2(ENC2A, ENC2B);
Encoder encTheta3(ENC3A, ENC3B);
Encoder encTheta4(ENC4A, ENC4B);

//HX711 scale1;
HX711_ADC epLoadcell(LC1_DOUT_PIN, LC1_SCK_PIN);

// Sensor data buffers
Buffer theta1, theta2, theta3, theta4;
Buffer force;
Buffer control;
Buffer desired;
// Target is set once and this is used to derive the desired value.
// All controllers that require a desired position will need to use 
// the data from the desired buffer.
float target;

// Additional buffers for storing the contoller related information. 
Buffer err;
Buffer errdiff;
Buffer errsum;

// Variable to hold the current PLUTO button state.
volatile byte marsButton = 1;

// Packet Counter.
uint16union_t packetNumber;

// run time
unsigned long startTime;
ulongunion_t runTime;

// Program status
byte streamType = DIAGNOSTICS;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
uint16union_t deviceError;

// Serial Reader object
SerialReader serReader;

// Out data buffer
OutDataBuffer4Float outPayload;

// Poition Control
float pcKp = 0.1;
float pcKd = 0.01;
float pcKi = 0.001;

/* Tempoary section : To be formated later */
Bounce bounce = Bounce();

// Timer interrupt for reading serial data
IntervalTimer readStream;

SoftwareSerial bt(0, 1);

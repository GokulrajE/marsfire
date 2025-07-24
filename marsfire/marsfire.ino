#include <Encoder.h>
#include "variable.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include <HX711_ADC.h>


void setup() {
  Serial.begin(115200);
  bt.begin(115200);

  // Set the read and write resolutions
  analogReadResolution(12);
  analogWriteResolution(PWMRESOLN);
  
  // Set up stuff.
  deviceSetUp();

  // Initialize variables.
  // Always use the setLimb function to change the limb.
  setLimb(NOLIMB);

  // Device button settiongs
  readStream.begin(readHandleIncomingMessage, 1000);

  // Set targets to invalid value 999.
  target = INVALID_TARGET;
  desired.add(INVALID_TARGET);

  // Initialize variable.
  streamType = DIAGNOSTICS; //SENSORSTREAM;
  stream = true;
  ctrlType = NONE;
  calib = NOCALIB;
  limbKinParam = NOLIMBKINPARAM;
  limbDynParam = NOLIMBDYNPARAM;
  deviceError.num = 0x0000;

  // Reset packet number and run time.
  packetNumber.num = 0;
  startTime = millis();
  runTime.num = 0;
  delTime = 1.0;

  // Last received heart beat time
  lastRxdHeartbeat = millis();
}

void loop() {
  // Check heartbeat
  checkHeartbeat();

  // Read and update sensor values.
  updateSensorData();

  // Handle errors.
  handleErrors();

  // // Send sensordata
  // if (stream) {
  //   writeSensorStream();
  // }

  // Update control
  updateControlLaw();

  // Send data out.
  writeSensorStream();

  // // Relax. You only need to work at around 200Hz
  // delay(2);
  packetNumber.num += 1;
  currMilliCount = millis();
  runTime.num = currMilliCount - startTime;
  delTime = (currMilliCount - prevMilliCount) / 1000.0;
  prevMilliCount = currMilliCount;

  // Update safety timers.
  updateSafetyTimers();

  // n = n + 1.0;

  //  updateCalibButton();
  //  updateEncoders();
  //  //Serial.println("2");
  //  updateLoadCells();
  //  //Serial.println("3");
  //  updateImu();
  //  readMarsButtonState();
  //  controller();
   //Serial.println("4");

   
   //Serial.println("5");
  //  readHandleIncomingMessage();

  //Serial.println(IMUtheta1);

//  Serial.println("1");
//  Serial.print(time_ellapsed);
//  Serial.print('\t');
//  Serial.print(theta1);
//  Serial.print('\t');
//  Serial.print(theta2);
//  Serial.print('\t');
//  Serial.print(theta3);
//  Serial.print('\t');
  // Serial.print(time_ellapsed);
  // Serial.print('\t');
  // Serial.println(ax1);

  // delay(10);
}

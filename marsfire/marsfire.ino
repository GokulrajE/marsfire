#include <Encoder.h>
#include "variable.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include <HX711_ADC.h>


void setup() {
  Serial.begin(115200);
  bt.begin(115200);

  SerialUSB.println("MARS Started");

  // Set the read and write resolutions
  analogReadResolution(12);
  analogWriteResolution(PWMRESOLN);
  
  // Set up stuff.
  deviceSetUp();
  // SerialUSB.println("Started.");

  // Initialize variables.
  // Always use the setLimb function to change the limb.
  setLimb(NOLIMB);

  // Device button settiongs
  readStream.begin(readHandleIncomingMessage, 100);

  // Set targets to invalid value 999.
  target = INVALID_TARGET;
  desired.add(INVALID_TARGET);

  // Initialize variable.
  streamType = DIAGNOSTICS; //SENSORSTREAM;
  stream = true;
  ctrlType = NONE;
  calib = NOCALIB;
  deviceError.num = 0x0000;

  // Reset packet number and run time.
  packetNumber.num = 0;
  startTime = millis();
  runTime.num = 0;
  delTime = 1.0;

  // Last received heart beat time
  lastRxdHeartbeat = millis();
  SerialUSB.println("All ready.");
}

void loop() {
  // Check heartbeat
  checkHeartbeat();

  // Read and update sensor values.
  updateSensorData();

  // Handle errors.
  handleErrors();
  
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
}

#include <Encoder.h>
#include "variable.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include <HX711_ADC.h>

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
MPU6050 mpu2(Wire1);
MPU6050 mpu3(Wire1);

void setup() {
  
  Serial.begin(115200);
  bt.begin(115200);

  analogReadResolution(12);
  analogWriteResolution(12);
  
  calibButtonSetup();
  encoderSetup();
  loadCellSetup();
  motorDataSetup();
  imuSetup();

// Device button settiongs
  button.attach(IO_SWITCH);
  button.interval(5);
  readStream.begin(readHandleIncomingMessage, 1000);
  //IMUupdate.begin(updateImu, 1000);
  //writeStream.begin(writeSensorStream, 5);

}

void loop() {
  
  time_ellapsed = millis()/1000.0;

  n = n + 1.0;

   updateCalibButton();
   updateEncoders();
   //Serial.println("2");
   updateLoadCells();
   //Serial.println("3");
   updateImu();
   readMarsButtonState();
   controller();
   //Serial.println("4");

   writeSensorStream();
   //Serial.println("5");
   readHandleIncomingMessage();

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

  delay(10);
}

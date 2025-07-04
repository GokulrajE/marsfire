
/*
 * Check if there is heartbeat.
 */
void checkHeartbeat() {
  // Check if a heartbeat was recently received.
  if (0.001 * (millis() - lastRxdHeartbeat) < MAX_HBEAT_INTERVAL) {
    // Everything is good. No heart beat related error.
    deviceError.num &= ~NOHEARTBEAT;
  } else {
    // No heartbeat received.
    // Setting error flag.
    deviceError.num |= NOHEARTBEAT;
  }
}

/*
 * Handles errors
 */
void handleErrors() {
  if (deviceError.num != 0) {
    // #if SERIALUSB_DEBUG
    //   SerialUSB.print("Error occured: ");
    //   SerialUSB.print(deviceError.num);
    //   SerialUSB.print("\n");
    // #endif
    setControlType(NONE);
  }
}

void _assignFloatUnionBytes(int inx, byte* bytes, floatunion_t* temp) {
  temp->bytes[0] = bytes[inx];
  temp->bytes[1] = bytes[inx + 1];
  temp->bytes[2] = bytes[inx + 2];
  temp->bytes[3] = bytes[inx + 3];
}

// Initial hardware set up the device.
void deviceSetUp()
{
  // 1. Calibration Button
  calibBounce.attach(CALIB_BUTTON);
  calibBounce.interval(5);
  
  // 2. Set up all encoders.
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  pinMode(ENC2A, INPUT_PULLUP);
  pinMode(ENC2B, INPUT_PULLUP);
  pinMode(ENC3A, INPUT_PULLUP);
  pinMode(ENC3B, INPUT_PULLUP);
  pinMode(ENC4A, INPUT_PULLUP);
  pinMode(ENC4B, INPUT_PULLUP);

  // 3. Set up the two loadcells
  scale1.begin();
  scale1.start(1, true);
  scale1.setCalFactor(LOACELL_CALIB_FACTOR);
  scale2.begin();
  scale2.start(1, true);
  scale2.setCalFactor(LOACELL_CALIB_FACTOR);

  // 4. Motor setup
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);

  // 5. IMU setup
  imuSetup();

  // 6. MARS button
  marsBounce.attach(MARS_BUTTON);
  marsBounce.interval(5);
}

/*
 * Read and update all sensor data.
 */
void updateSensorData()
{
  // 1. Read the force sensors.
  scale1.update();
  force1 = -scale1.getData();
  scale2.update();
  force2 = scale2.getData();

  // 2. Read the encoder data.  
  theta1 = limbScale1 * (read_angle1() + offset1);
  theta2 = limbScale2 * (read_angle2() + offset2);
  theta3 = limbScale3 * (read_angle3() + offset3);
  theta4 = limbScale4 * (read_angle4() + offset4);

  // 3. Read buttons.
  marsBounce.update();
  marsButton = marsBounce.read();
  calibBounce.update();
  calibButton = calibBounce.read();
  devButtons = (calibButton << 1) | marsButton;

  // 4. Read IMU data.
  updateImu();
}


void setSupport (int sz, int strtInx, byte* payload)
{
  int inx = strtInx;
  floatunion_t temp;
  _assignFloatUnionBytes(inx, payload, &temp);
  des1 = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  des2 = temp.num;
  inx += 4;
   _assignFloatUnionBytes(inx, payload, &temp);
  des3 = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  PCParam = temp.num;
  inx += 4; 
}


byte getProgramStatus(byte dtype) {
  // X | DATA TYPE | DATA TYPE | DATA TYPE | CONTROL TYPE | CONTROL TYPE | CONTROL TYPE | CALIB
  return ((dtype << 4) | (ctrlType << 1) | (calib & 0x01));
}


byte getAdditionalInfo(void) {
  // X | X | CALIB | MARS | X | X | CURR LIMB | CURR LIMB
  return (devButtons << 4) | currLimb;
}

void setLimb(byte limb) {
  currLimb = limb;
  if (currLimb == LEFT) {
    limbScale1 = -1.0;
    limbScale2 = -1.0;
    limbScale3 = -1.0;
    limbScale4 = -1.0;
  } else {
    limbScale1 = 1.0;
    limbScale2 = 1.0;
    limbScale3 = 1.0;
    limbScale4 = 1.0;
  }
}

// void readMarsButtonState(void) {
//    button.update();                  // Update the Bounce instance
//    marsButton = button.read();
// }

// void updateEncoders()
// {
//   theta1Enc = read_angle1();
//   theta2Enc = read_angle2();
//   theta3Enc = read_angle3();
//   theta4Enc = read_angle4();
  
//   theta1 = theta1Enc + offset1;
//   theta2 = theta2Enc + offset2;
//   theta3 = theta3Enc + offset3;
//   theta4 = theta4Enc + offset4;
 
// }

// void updateLoadCells()
// {
//   scale1.update();
//   force1 = -scale1.getData();

//   scale2.update();
//   force2 = scale2.getData();

// }

float read_angle1()
{
  _enccount1 = angle1.read();
  if (_enccount1 >= ENC1MAXCOUNT) {
    angle1.write(_enccount1 - ENC1MAXCOUNT);
  } else if (_enccount1 <= - ENC1MAXCOUNT) {
    angle1.write(_enccount1 + ENC1MAXCOUNT);
  }
  return ENC1COUNT2DEG * _enccount1;
}

float read_angle2()
{
  _enccount2 = angle2.read();
  if (_enccount2 >= ENC1MAXCOUNT) {
    angle2.write(_enccount2 - ENC2MAXCOUNT);
  } else if (_enccount2 <= - ENC2MAXCOUNT) {
    angle2.write(_enccount2 + ENC2MAXCOUNT);
  }
  return ENC2COUNT2DEG * _enccount2;
}

float read_angle3()
{
  _enccount3 = angle3.read();
  if (_enccount3 >= ENC3MAXCOUNT) {
    angle3.write(_enccount3 - ENC3MAXCOUNT);
  } else if (_enccount3 <= - ENC3MAXCOUNT) {
    angle3.write(_enccount3 + ENC3MAXCOUNT);
  }
  return ENC3COUNT2DEG * _enccount3;
}

float read_angle4()
{
  _enccount4 = angle4.read();
  if (_enccount4 >= ENC4MAXCOUNT) {
    angle4.write(_enccount4 - ENC4MAXCOUNT);
  } else if (_enccount4 <= - ENC4MAXCOUNT) {
    angle4.write(_enccount4 + ENC4MAXCOUNT);
  }
  return ENC4COUNT2DEG * _enccount4;
}

void imuSetup()
{
  Wire.begin();
  mpu.setAddress(0x68);
  byte status = mpu.begin();
  Wire1.begin();
  mpu2.setAddress(0x68);
  mpu2.begin();
  mpu3.setAddress(0x69);
  mpu3.begin();

  while(status!=0){ } // stop everything if could not connect to MPU6050
  delay(1000);

  mpu.calcOffsets(true, false); // gyro and accelero
  mpu2.calcOffsets(true, false);
  mpu3.calcOffsets(true, false);
}

void updateImu()
{
  mpu.update();
  mpu2.update();
  mpu3.update();
  
   ax1 = mpu.getAccX();
   ay1 = mpu.getAccY();
   az1 = mpu.getAccZ();

   ax2 = mpu2.getAccX();
   ay2 = mpu2.getAccY();
   az2 = mpu2.getAccZ();

   ax3 = mpu3.getAccX();
   ay3 = mpu3.getAccY();
   az3 = mpu3.getAccZ();
   
   norm1 = pow(ax1*ax1 + ay1*ay1 + az1*az1, 0.5);
   norm2 = pow(ax2*ax2 + ay2*ay2 + az2*az2, 0.5);
   norm3 = pow(ax3*ax3 + ay3*ay3 + az3*az3, 0.5);

   IMUtheta1 = -180/3.14*(asin(ax1/norm1) + asin(ax2/norm2) + asin(az3/norm3))/3;
   IMUtheta2 = -180/3.14*atan2(az1, -ay1);
   IMUtheta3 = 180/3.14*(-atan2(az2, -ay2)) - IMUtheta2;
   IMUtheta4 = 180/3.14*atan2(-ax3, ay3) - IMUtheta2 - IMUtheta3;
}

// void calibButtonSetup()
// {
//   pinMode(CALIB_BUTTON, INPUT_PULLUP);
// }

void updateCalibButton()
{
  calibButtonState = 1.0 * digitalRead(CALIB_BUTTON);
  // Serial.println(digitalRead(calib_button_pin));
}


void _assignFloatUnionBytes(int inx, byte* bytes, floatunion_t* temp) {
  temp->bytes[0] = bytes[inx];
  temp->bytes[1] = bytes[inx + 1];
  temp->bytes[2] = bytes[inx + 2];
  temp->bytes[3] = bytes[inx + 3];
}

// Initial hardware set up the device.
void deviceSetUp() {
  // 1. Calibration and MARS Buttons
  pinMode(CALIB_BUTTON, INPUT_PULLUP);
  pinMode(MARS_BUTTON, INPUT_PULLUP);
  
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
  pinMode(MOTOR_PWM, OUTPUT);

  // 5. IMU setup
  imuSetup();
}

/*
 * Read and update all sensor data.
 */
void updateSensorData() {
  // 1. Read the force sensors.
  scale1.update();
  epForce = -scale1.getData();
  // scale2.update();
  // force2 = scale2.getData();

  // 2. Read the encoder data.
  // Update previous values.
  theta1Prev = theta1;
  theta2Prev = theta2;
  theta3Prev = theta3;
  theta4Prev = theta4;
  // Read the current values.
  theta1 = limbAngleScale * (read_angle1() + theta1Offset);
  theta2 = limbAngleScale * (read_angle2() + theta2Offset);
  theta3 = limbAngleScale * (read_angle3() + theta3Offset);
  theta4 = limbAngleScale * (read_angle4() - theta4Offset);
  // Angle in radians.
  theta1r = DEG2RAD(theta1);
  theta2r = DEG2RAD(theta2);
  theta3r = DEG2RAD(theta3);
  theta4r = DEG2RAD(theta4);
  // Cosine and Sine terms.
  cos1 = cos(theta1r);
  cos2 = cos(theta2r);
  cos3 = cos(theta3r);
  cos4 = cos(theta4r);
  sin1 = sin(theta1r);
  sin2 = sin(theta2r);
  sin3 = sin(theta3r);
  sin4 = sin(theta4r);
  // 2a. Update actual angle buffer
  actual.add(theta1);
  // 2b. Compute the angular velocities
  omega1 = (theta1 - theta1Prev) / delTime;
  omega2 = (theta2 - theta2Prev) / delTime;
  omega3 = (theta3 - theta3Prev) / delTime;
  omega4 = (theta4 - theta4Prev) / delTime;

  // 3. Read buttons.
  marsButton = updatButtonPressValue(MARS_BUTTON, &marsBounceCount, marsButton);
  calibButton = updatButtonPressValue(CALIB_BUTTON, &calibBounceCount, calibButton);
  devButtons = (calibButton << 1) | marsButton;

  // 4. Read IMU data.
  updateImu();

  // 5. Compute robot endpoint position.
  updateEndpointPosition();

  // 6. Compute the human limb joint angles. 


  // 7. Update the current torque and rate of change of torque.
  torque = epForce * sqrt(xEp * xEp + yEp * yEp) / 1000;
  dTorque = (torque - torquePrev) / delTime;
  torquePrev = torque;
}


void setSupport(int sz, int strtInx, byte* payload) {
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
  // CMD_STATUS | CMD_STATUS | CALIB | MARS | LIMBDYNPARAM | LIMBKINPARAM | CURR LIMB | CURR LIMB
  return (cmdStatus << 6) | (devButtons << 4) | (limbDynParam << 3) | (limbKinParam << 2) | currLimb;
}

void setLimb(byte limb) {
  currLimb = limb;
  if (currLimb == LEFT) {
    limbAngleScale = -1.0;
    limbControlScale = 1.0;
  } else {
    limbAngleScale = 1.0;
    limbControlScale = -1.0;
  }
}

bool isWithinRange(float val, float minVal, float maxVal) {
  if (val < minVal) return false;
  else if (val  > maxVal) return false;
  return true; 
}

byte updatButtonPressValue(byte pinNo, int8_t* bounceCount, byte buttonValue) {
  byte _btnval = digitalRead(pinNo);
  // Change in button state.
  if (buttonValue == _btnval) return buttonValue;
  // Change in state.
  if (_btnval == 0) {
    if (*bounceCount >= 0) *bounceCount = -1;
    else (*bounceCount)--;
  } else {
    if (*bounceCount <= 0) *bounceCount = 1;
    else (*bounceCount)++;
  }
  // Check if bounceCount has crossed the threshold.
  if (*bounceCount <= -BOUNCE_THRESHOLD) return 0;
  else if (*bounceCount >= BOUNCE_THRESHOLD) return 1;
  else return buttonValue;
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

float read_angle1() {
  _enccount1 = angle1.read();
  if (_enccount1 >= ENC1MAXCOUNT) {
    angle1.write(_enccount1 - ENC1MAXCOUNT);
  } else if (_enccount1 <= -ENC1MAXCOUNT) {
    angle1.write(_enccount1 + ENC1MAXCOUNT);
  }
  return ENC1COUNT2DEG * _enccount1;
}

float read_angle2() {
  _enccount2 = angle2.read();
  if (_enccount2 >= ENC1MAXCOUNT) {
    angle2.write(_enccount2 - ENC2MAXCOUNT);
  } else if (_enccount2 <= -ENC2MAXCOUNT) {
    angle2.write(_enccount2 + ENC2MAXCOUNT);
  }
  return ENC2COUNT2DEG * _enccount2;
}

float read_angle3() {
  _enccount3 = angle3.read();
  if (_enccount3 >= ENC3MAXCOUNT) {
    angle3.write(_enccount3 - ENC3MAXCOUNT);
  } else if (_enccount3 <= -ENC3MAXCOUNT) {
    angle3.write(_enccount3 + ENC3MAXCOUNT);
  }
  return ENC3COUNT2DEG * _enccount3;
}

float read_angle4() {
  _enccount4 = angle4.read();
  if (_enccount4 >= ENC4MAXCOUNT) {
    angle4.write(_enccount4 - ENC4MAXCOUNT);
  } else if (_enccount4 <= -ENC4MAXCOUNT) {
    angle4.write(_enccount4 + ENC4MAXCOUNT);
  }
  return ENC4COUNT2DEG * _enccount4;
}

void imuSetup() {
  Wire.begin();
  mpu.setAddress(0x68);
  byte status = mpu.begin();
  Wire1.begin();
  mpu2.setAddress(0x68);
  mpu2.begin();
  mpu3.setAddress(0x69);
  mpu3.begin();

  while (status != 0) {}  // stop everything if could not connect to MPU6050
  delay(1000);

  mpu.calcOffsets(true, false);  // gyro and accelero
  mpu2.calcOffsets(true, false);
  mpu3.calcOffsets(true, false);
}

void updateImu() {
  float ax1, ay1, az1, ax2, ay2, az2, ax3, ay3, az3;
  float norm1, norm2, norm3, norm_sum;

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

  norm1 = pow(ax1 * ax1 + ay1 * ay1, 0.5);
  norm2 = pow(ax2 * ax2 + ay2 * ay2, 0.5);
  norm3 = pow(ay3 * ay3 + az3 * az3, 0.5);
  norm_sum = norm1 + norm2;

  // Theta 1.
  imuTheta1 = (norm1 * RAD2DEG(atan2(ax1, ay1))
               + norm2 * RAD2DEG(atan2(ax2, ay2))) / norm_sum;
  imuTheta1 -= IMU1OFFSET; 
  imuTheta2 = RAD2DEG(atan2(-az1, norm1));
  imuTheta2 -= IMU2OFFSET; 
  imuTheta3 = RAD2DEG(atan2(-az2, norm2)) - imuTheta2;
  imuTheta3 -= IMU3OFFSET; 
  imuTheta4 = RAD2DEG(atan2(-ax3, norm3)) - imuTheta2 - imuTheta3;
  imuTheta4 -= IMU4OFFSET; 

  // Update the IMU angle bytes
  imu1Byte = (int8_t) imuTheta1;
  imu2Byte = (int8_t) imuTheta2;
  imu3Byte = (int8_t) imuTheta3;
  imu4Byte = (int8_t) imuTheta4;
}

// void calibButtonSetup()
// {
//   pinMode(CALIB_BUTTON, INPUT_PULLUP);
// }

void updateCalibButton() {
  calibButtonState = 1.0 * digitalRead(CALIB_BUTTON);
  // Serial.println(digitalRead(calib_button_pin));
}

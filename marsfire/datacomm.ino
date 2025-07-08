
//#define SerialPort SerialUSB
//#include "variable.h"
//#include "CustomDS.h"



void writeSensorStream()
{
  // Format:
  // 255 | 255 | No. of bytes | Status | Error Val 1 | Error Val 2 | ...
  // Payload | Chksum
  byte header[] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 };
  byte chksum = 0xFE;
  byte _temp;


  //Out data buffer
  outPayload.newPacket();
  outPayload.add(theta1);
  outPayload.add(theta2);
  outPayload.add(theta3);
  outPayload.add(theta4);
  outPayload.add(force1);

  // Send packet.
  header[2] = (4                      // Four headers
               + 2                    // Packet number int16
               + 4                    // Run time
               + outPayload.sz() * 4  // Float sensor data
               + 2                    // Limb lengths
               + 2                    // Limb weights
               + 1                    // Shoulder position
               + 4                    // IMU angles
               + 1                    // Checksum
               );
  header[3] = getProgramStatus(streamType);
  header[4] = deviceError.bytes[0];
  header[5] = deviceError.bytes[1];
  header[6] = getAdditionalInfo();
  chksum += header[2] + header[3] + header[4] + header[5] + header[6];

  //Send header
  bt.write(header[0]);
  bt.write(header[1]);
  bt.write(header[2]);
  bt.write(header[3]);
  bt.write(header[4]);
  bt.write(header[5]);
  bt.write(header[6]);
  
  // Send packet number
  for (int i = 0; i < 2; i++) {
    bt.write(packetNumber.bytes[i]);
    chksum += packetNumber.bytes[i];
  }

  // Send current run time
  for (int i = 0; i < 4; i++) {
    bt.write(runTime.bytes[i]);
    chksum += runTime.bytes[i];
  }

  // Send the floats
  for (int i = 0; i < outPayload.sz() * 4; i++) {
    _temp = outPayload.getByte(i);
    bt.write(_temp);
    chksum += _temp;
  }

  // Send the human limb parameters
  bt.write(uaLByte);
  chksum += uaLByte;
  bt.write(faLByte);
  chksum += faLByte;
  bt.write(uaWByte);
  chksum += uaWByte;
  bt.write(faWByte);
  chksum += faWByte;
  bt.write(shZByte);
  chksum += shZByte;

  // Send the IMU angles
  bt.write(imu1Byte);
  chksum += imu1Byte;
  bt.write(imu2Byte);
  chksum += imu2Byte;
  bt.write(imu3Byte);
  chksum += imu3Byte;
  bt.write(imu4Byte);
  chksum += imu4Byte;

  bt.write(chksum);
  bt.flush();
}


// Read and handle incoming messages.
// Many commands will be ignored if there is a device error.
void readHandleIncomingMessage() {
  byte _details;
  int plSz = serReader.readUpdate();

  // Read handle incoming data
  if (plSz > 0) {
    _details = serReader.payload[1];
    // Handle new message.
    // Check the command type.
    switch (serReader.payload[0]) {
      case START_STREAM:
        stream = true;
        streamType = SENSORSTREAM;
        break;
      case STOP_STREAM:
        stream = false;
        break;
      case SET_DIAGNOSTICS:
        stream = true;
        streamType = DIAGNOSTICS;
        break;
      // case SET_CONTROL_TYPE:
      //   #if SERIALUSB_DEBUG
      //     // Debug print
      //     SerialUSB.print("\nControl Type: ");
      //     SerialUSB.print(currMech);
      //     SerialUSB.print(" ");
      //     SerialUSB.print(deviceError.num);
      //     SerialUSB.print(" ");
      //     SerialUSB.print(_details);
      //     SerialUSB.print(" ");
      //   #endif
      //   // This can be set only if there is not error.
      //   if (deviceError.num != 0) break;
      //   // This can only be set if a mechanism is selected, 
      //   // and calibration has been done.
      //   if ((currMech == NOMECH) || (calib == NOCALIB)) break;
      //   // No Error, Mechanism set and calibrated.
      //   setControlType(_details);
      //   #if SERIALUSB_DEBUG
      //     SerialUSB.print(ctrlType);
      //     SerialUSB.print("\n");
      //   #endif
      //   break;
      // case SET_CONTROL_TARGET:
      //   #if SERIALUSB_DEBUG
      //     SerialUSB.print("Control Target: ");
      //     SerialUSB.print(currMech);
      //     SerialUSB.print(" ");
      //     SerialUSB.print(deviceError.num);
      //     SerialUSB.print("\n");
      //   #endif
      //   // This can be set only if there is not error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   // Check if the current control type is POSITION or TORQUE.
      //   if ((ctrlType == POSITION)
      //       || (ctrlType == POSITIONLINEAR)
      //       || (ctrlType == TORQUE)
      //       || (ctrlType == TORQUELINEAR)) {
      //     // Set target.
      //     setTarget(serReader.payload, 1, ctrlType);
      //     // Initial time.
      //     initTime = runTime.num / 1000.0f + strtTime;
      //     // Reset the control hold value.
      //     setControlHold(CONTROL_FREE);
      //     #if SERIALUSB_DEBUG
      //       SerialUSB.print("Target set: ");
      //       SerialUSB.print(strtPos);
      //       SerialUSB.print(",");
      //       SerialUSB.print(strtTime);
      //       SerialUSB.print(",");
      //       SerialUSB.print(target);
      //       SerialUSB.print(",");
      //       SerialUSB.print(reachDur);
      //       SerialUSB.print(",");
      //       SerialUSB.print(initTime);
      //       SerialUSB.print("\n");
      //     #endif
      //   }
      //   break;
      // case SET_CONTROL_BOUND:
      //   // This can be set only if there is no error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   // Check if the current control type is POSITION.
      //   ctrlBound = 0.0;
      //   if ((ctrlType == POSITION)
      //       || (ctrlType == POSITIONLINEAR)
      //       || (ctrlType == POSITIONAAN)) {
      //     ctrlBound = _details / 255.0;
      //   }
      //   break;
      // case SET_CONTROL_DIR:
      //   // This can be set only if there is not error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   // Check if the current control type is POSITIONAAN.
      //   ctrlDir = 0;
      //   if (ctrlType == POSITIONAAN) {
      //     ctrlDir = _details;
      //   }
      //   break;
      // case SET_CONTROL_GAIN:
      //   // Reset controller gain.
      //   ctrlGain = 0;
      //   // Set the gain of the controller.
      //   // This can be set only if there is not error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   ctrlGain = (uint8_t) _details;
      //   break;
      // case SET_AAN_TARGET:
      //   // This can be set only if there is no error.
      //   if (deviceError.num != 0) break;
      //   // No Error.
      //   if (ctrlType != POSITIONAAN) break;
      //   // Set AAN Target.
      //   setAANTarget(serReader.payload, 1);
      //   // Set Control Direction.
      //   ctrlDir = target >= strtPos ? +1 : -1;
      //   // Initial time.
      //   initTime = runTime.num / 1000.0f + strtTime;
      //   #if SERIALUSB_DEBUG
      //     SerialUSB.print(strtPos);
      //     SerialUSB.print(",");
      //     SerialUSB.print(strtTime);
      //     SerialUSB.print(",");
      //     SerialUSB.print(target);
      //     SerialUSB.print(",");
      //     SerialUSB.print(reachDur);
      //     SerialUSB.print(",");
      //     SerialUSB.print(initTime);
      //     SerialUSB.print("\n");
      //   #endif
      //   break;
      // case HOLD_CONTROL:
      //   // This can be set only if there is no error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   // Check if the current control type is POSITION.
      //   setControlHold(CONTROL_FREE);
      //   if ((ctrlType == POSITION)
      //       || (ctrlType == POSITIONLINEAR)
      //       || (ctrlType == POSITIONAAN)) {
      //     setControlHold(CONTROL_HOLD);
      //   }
      //   break;
      // case DECAY_CONTROL:
      //   // This can be set only if there is no error.
      //   if (deviceError.num != 0) break;
      //   // No Error
      //   // Check if the current control type is POSITION.
      //   setControlHold(CONTROL_FREE);
      //   if ((ctrlType == POSITION)
      //       || (ctrlType == POSITIONLINEAR)
      //       || (ctrlType == POSITIONAAN)
      //       || (ctrlType == OBJECTSIM)) {
      //     setControlHold(CONTROL_DECAY);
      //   }
      //   break;
      // case SET_OBJECT_PARAM:
      //   if (deviceError.num != 0) break;
      //   if (ctrlType != OBJECTSIM) break;
      //   setObjectParams(serReader.payload, 1);
      //   setControlHold(CONTROL_FREE);
      //   break;
      // case GET_OBJECT_PARAM:
      //   // Send the current firmware version.
      //   sendObjectParams();
      //   break;
      // case RESET_AAN_TARGET:
      //   target = INVALID_TARGET;
      //   ctrlDir = 0;
      //   break;
      case SET_LIMB:
        // This can only be set if there is no error.
        if (deviceError.num != 0) break;
        // This can only be set if the control is NONE.
        if (ctrlType != NONE) break;
        // Reset calibration
        calib = NOCALIB;
        // Make sure the input limb is one of the valid options.
        setLimb(isValidLimb(_details) ? _details : NOLIMB);
        break;
      case CALIBRATE:
        // This can be set only if there is no error.
        if (deviceError.num != 0) break;
        // This can only eb set if the control is NONE.
        if (ctrlType != NONE) break;
        // Check if the limb has been set.
        if (currLimb == NOLIMB) break;
        // Reset calibration
        // Check the calibration value
        calib = NOCALIB;
        // Ensure that the IMU angles are not too off.
        if ((imuTheta1 < CALIB_IMU_ANGLE_MIN) || 
            (imuTheta2 < CALIB_IMU_ANGLE_MIN) || 
            (imuTheta3 < CALIB_IMU_ANGLE_MIN) || 
            (imuTheta4 < CALIB_IMU_ANGLE_MIN) || 
            (imuTheta1 > CALIB_IMU_ANGLE_MAX) || 
            (imuTheta2 > CALIB_IMU_ANGLE_MAX) || 
            (imuTheta3 > CALIB_IMU_ANGLE_MAX) || 
            (imuTheta4 > CALIB_IMU_ANGLE_MAX)) break;
        // Set the offset angles.
        theta1Offset = imuTheta1;
        theta2Offset = imuTheta2;
        theta3Offset = imuTheta3;
        theta4Offset = imuTheta4;
        // Reset encoder counts.
        angle1.write(0);
        angle2.write(0);
        angle3.write(0);
        angle4.write(0);
        calib = YESCALIB;
        break;
      case GET_VERSION:
        stream = false;
        // Send the current firmware version.
        sendVersionDetails();
        break;
      case HEARTBEAT:
        lastRxdHeartbeat = millis();
        break;
      case RESET_PACKETNO:
        packetNumber.num = 0;
        startTime = millis();
        runTime.num = 0;
        break;
    }
    serReader.payloadHandled();
  }
}


void sendVersionDetails() {
  // Format:
  // 255 | 255 | No. of bytes | Status | Error Val 1 | Error Val 2 | ...
  // [Current Mechanism][isActuated] | Payload | Chksum
  byte header[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 };
  byte chksum = 0xFE;

  // Send packet.
  header[2] = (4                      // Four headers
               + strlen(fwVersion)    // Version string length
               + 1                    // Comma separator
               + strlen(deviceId)     // Deviice ID string length
               + 1                    // Comma separator
               + strlen(compileDate)  // Compilation date.
               + 1                    // Checksum
  );
  header[3] = getProgramStatus(VERSION);
  header[4] = deviceError.bytes[0];
  header[5] = deviceError.bytes[1];
  header[6] = getAdditionalInfo();
  chksum += header[2] + header[3] + header[4] + header[5] + header[6];

  // Send the header.
  bt.write(header[0]);
  bt.write(header[1]);
  bt.write(header[2]);
  bt.write(header[3]);
  bt.write(header[4]);
  bt.write(header[5]);
  bt.write(header[6]);

  // Send Device ID
  for (unsigned int i = 0; i < strlen(deviceId); i++) {
    bt.write(deviceId[i]);
    chksum += deviceId[i];
  }
  bt.write(',');
  chksum += ',';
  // Send firmware version
  for (unsigned int i = 0; i < strlen(fwVersion); i++) {
    bt.write(fwVersion[i]);
    chksum += fwVersion[i];
  }
  bt.write(',');
  chksum += ',';
  // Send firmware compilation date
  for (unsigned int i = 0; i < strlen(compileDate); i++) {
    bt.write(compileDate[i]);
    chksum += compileDate[i];
  }
  // Send Checksum
  bt.write(chksum);
  bt.flush();
}

/*
 * Check if the given limb is valid.
 */
bool isValidLimb(byte limbval) {
  return (
    (limbval == NOLIMB) ||
    (limbval == RIGHT) ||
    (limbval == LEFT)
  );
}


// void readHandleIncomingMessage() {
//     int plSz = serReader.readUpdate();
//     payLoadSize = plSz*1.0;
//     int stInx = 0;

//     // Read handle incoming data
//     if (plSz > 0 && plSz == 16) {
//         // Handle new message.
//         // Check the command type.

//         setSupport(plSz, stInx, serReader.payload);

//        serReader.payloadHandled();
//        Serial.clear();
//     }
// }

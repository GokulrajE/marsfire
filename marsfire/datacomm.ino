
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


void readHandleIncomingMessage() {
    int plSz = serReader.readUpdate();
    payLoadSize = plSz*1.0;
    int stInx = 0;

    // Read handle incoming data
    if (plSz > 0 && plSz == 16) {
        // Handle new message.
        // Check the command type.

        setSupport(plSz, stInx, serReader.payload);

       serReader.payloadHandled();
       Serial.clear();
    }
}

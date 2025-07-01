
//#define SerialPort SerialUSB
//#include "variable.h"
//#include "CustomDS.h"



void writeSensorStream()
{
  byte header[] = {0xFF, 0xFF, 0x00};
  byte chksum = 0xFE;
  byte _temp;


  //Out data buffer
  outPayload.newPacket();

  outPayload.add(theta1);
  outPayload.add(theta2);
  outPayload.add(theta3);
  outPayload.add(theta4);
  outPayload.add(force1);
  outPayload.add(calibButtonState);
  outPayload.add(des1);
  outPayload.add(des2);
  outPayload.add(des3);
  outPayload.add(PCParam);
  outPayload.add(shx);
  outPayload.add(shy);
  outPayload.add(shz);
  outPayload.add(upperArm);
  outPayload.add(foreArm);
  outPayload.add(W1);
  outPayload.add(W2);
  outPayload.add(IMUtheta1);
  outPayload.add(IMUtheta2);
  outPayload.add(IMUtheta3);
  outPayload.add(IMUtheta4);
  //if need to send imu RawData
  // outPayload.add(ax1);
  // outPayload.add(ay1);
  // outPayload.add(az1);
  // outPayload.add(ax2);
  // outPayload.add(ay2);
  // outPayload.add(az2);
  // outPayload.add(ax3);
  // outPayload.add(ay3);
  // outPayload.add(az3);

//  outPayload.add(elby);
//  outPayload.add(elbz);
//  outPayload.add(shx);
//  outPayload.add(shy);
//  outPayload.add(shz);
//  outPayload.add(endx);
//  outPayload.add(endy);
//  outPayload.add(endz);
//  outPayload.add(upperArm);
//  outPayload.add(foreArm);
//  outPayload.add(W1);
//  outPayload.add(W2);

  //  send packet
  header[2] = outPayload.sz() * 4 + 1+1;
  chksum += header[2];

  //Send header
//  Serial1.write(header[0]);
//  Serial1.write(header[1]);
//  Serial1.write(header[2]);
//  Serial1.write(header[3]);
  //
    bt.write(header[0]);
    bt.write(header[1]);
    bt.write(header[2]);

  //
    //Serial.println(header[0]);
    //Serial.println(header[1]);
   //Serial.println(header[2]);


  // Send payload
  for (int i = 0; i < outPayload.sz() * 4; i++) {
    _temp = outPayload.getByte(i);
//    Serial1.write(_temp);
        bt.write(_temp);
    chksum += _temp;
  }
  //Send checksum
//  Serial1.write(chksum);
     bt.write(marsButton);
     chksum += marsButton;

    bt.write(chksum);
    bt.flush();
    //serial.println(chksum);

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

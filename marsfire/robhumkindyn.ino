/* Functions to handle robot and human limb related computations for control, 
 * data communication for the MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */

byte setHumanLimbKinParams(byte* payload, int strtInx) {
  int inx = strtInx;
  floatunion_t temp;
  bool valRangeCheck;
  // Upper arm length
  _assignFloatUnionBytes(inx, payload, &temp);
  // SerialUSB.print(" UA: ");
  // SerialUSB.print(temp.num);
  valRangeCheck = isWithinRange(temp.num, MIN_UA_LENGTH, MAX_UA_LENGTH);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  uaLength = temp.num;
  uaL2 = uaLength * uaLength;
  inx += 4;
  // Forearm length
  _assignFloatUnionBytes(inx, payload, &temp);
  // SerialUSB.print(" FA: ");
  // SerialUSB.print(temp.num);
  valRangeCheck = isWithinRange(temp.num, MIN_FA_LENGTH, MAX_FA_LENGTH);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  faLength = temp.num;
  faL2 = faLength * faLength;
  uafaL = uaLength * faLength;
  inx += 4;
  // Shoulder position
  _assignFloatUnionBytes(inx, payload, &temp);
  // SerialUSB.print("  Z: ");
  // SerialUSB.print(temp.num);
  valRangeCheck = isWithinRange(temp.num, MIN_SHLDR_Z_POS, MAX_SHLDR_Z_POS);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  shPosZ = temp.num;
  // Update the human limb parameter bytes.
  // updateHumanLimbKinParamBytes();
  return YESLIMBKINPARAM;
}

byte setHumanLimbDynParams(byte* payload, int strtInx) {
  int inx = strtInx;
  floatunion_t temp;
  bool valRangeCheck;
  // Upper arm weight
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_UA_WEIGHT, MAX_UA_WEIGHT);
  if (!valRangeCheck) return NOLIMBDYNPARAM;
  uaWeight = temp.num;
  inx += 4;
  // Forearm weight
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_FA_WEIGHT, MAX_FA_WEIGHT);
  if (!valRangeCheck) return NOLIMBDYNPARAM;
  faWeight = temp.num;
  // Update the human limb parameter bytes.
  // updateHumanLimbDynParamBytes();
  return YESLIMBDYNPARAM;
}

// void updateHumanLimbKinParamBytes()
// {
//   // Upper arm parameters
//   uaLByte = (byte) 255 * (uaLength - MIN_UA_LENGTH) / (MAX_UA_LENGTH - MIN_UA_LENGTH);
//   // Forearm parameters
//   faLByte = (byte) 255 * (faLength - MIN_FA_LENGTH) / (MAX_FA_LENGTH - MIN_FA_LENGTH);
//   // Shoulder Z position
//   shZByte = (byte) 255 * (shPosZ - MIN_SHLDR_Z_POS) / (MAX_SHLDR_Z_POS - MIN_SHLDR_Z_POS);
// }

// void updateHumanLimbDynParamBytes()
// {
//   // Upper arm parameters
//   uaWByte = (byte) 255 * (uaWeight - MIN_UA_WEIGHT) / (MAX_UA_WEIGHT - MIN_UA_WEIGHT);
//   // Forearm parameters
//   faWByte = (byte) 255 * (faWeight - MIN_FA_WEIGHT) / (MAX_FA_WEIGHT - MIN_FA_WEIGHT);
// }

void updateEndpointPosition()
{
  float _temp = L1 * cos2 + L2 * cos(theta2r + theta3r);
  xEp = cos1 * _temp;
  yEp = sin1 * _temp;
  zEp = - L1 * sin2 - L2 * sin(theta2r + theta3r);
}

void updateHumanJointAngles()
{
  // Only if the kinematic parameters are set.
  if (limbKinParam != YESLIMBKINPARAM) {
    phi1 = 0;
    phi2 = 0;
    phi3 = 0;
    return;
  }
  // We can do inverse kinematics.
  float _zEp = zEp - shPosZ;
  phi1 = atan2(yEp, xEp);

  // Transform to the robot plane.
  float _xEp = xEp * cos(-phi1) - yEp * sin(-phi1);
  
  // Compute phi3.
  float _r2 = (_xEp * _xEp) + (_zEp * _zEp);
  float _ratio1 = (uaL2 + faL2 - _r2) / (2 * uafaL);
  _ratio1 = min(1.0, max(_ratio1, -1.0));
  phi3 = acos(_ratio1) - PI;

  // Compute phi2.
  float _ratio2 = (_r2 + uaL2 - faL2) / (2 * uaLength * sqrt(_r2));
  _ratio2 = min(1.0, max(_ratio2, -1.0));
  phi2 = acos(_ratio2) - atan2(_zEp, _xEp);
}

float getMarsGravityCompensationTorque()
{
  return (marsGCParam[0] * cos1
          + marsGCParam[1] * sin1
          + marsGCParam[2] * cos2 * sin1
          + marsGCParam[3] * cos2 * cos3 * sin1
          + marsGCParam[4] * sin1 * sin2
          + marsGCParam[5] * cos3 * sin1 * sin2
          + marsGCParam[6] * cos2 * sin1 * sin3
          + marsGCParam[7] * sin1 * sin2 * sin3);
}

float getHumanJointTorque()
{
  float _sp1 = sin(phi1);
  float _cp2 = cos(phi2);
  float _cp23 = cos(phi2 - phi3);
  return uaWeight * _sp1 * _cp2 + faWeight * _sp1 * _cp23;
}
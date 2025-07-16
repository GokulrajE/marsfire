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
  valRangeCheck = isWithinRange(temp.num, MIN_UA_LENGTH, MAX_UA_LENGTH);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  uaLength = temp.num;
  inx += 4;
  // Forearm length
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_FA_LENGTH, MAX_FA_LENGTH);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  faLength = temp.num;
  inx += 4;
  // Shoulder position
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_SHLDR_Z_POS, MAX_SHLDR_Z_POS);
  if (!valRangeCheck) return NOLIMBKINPARAM;
  shPosZ = temp.num;
  // Update the human limb parameter bytes.
  updateHumanLimbKinParamBytes();
  return YESLIMBKINPARAM;
}

byte setHumanLimbDynParams(byte* payload, int strtInx) {
  int inx = strtInx;
  floatunion_t temp;
  bool valRangeCheck;
  // Upper arm weight
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_UA_LENGTH, MAX_UA_LENGTH);
  if (!valRangeCheck) return NOLIMBDYNPARAM;
  uaLength = temp.num;
  inx += 4;
  // Forearm weight
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_FA_LENGTH, MAX_FA_LENGTH);
  if (!valRangeCheck) return NOLIMBDYNPARAM;
  faLength = temp.num;
  inx += 4;
  // Shoulder position
  _assignFloatUnionBytes(inx, payload, &temp);
  valRangeCheck = isWithinRange(temp.num, MIN_SHLDR_Z_POS, MAX_SHLDR_Z_POS);
  if (!valRangeCheck) return NOLIMBDYNPARAM;
  shPosZ = temp.num;
  // Update the human limb parameter bytes.
  updateHumanLimbDynParamBytes();
  return YESLIMBDYNPARAM;
}

void updateHumanLimbKinParamBytes()
{
  // Upper arm parameters
  uaLByte = (byte) 255 * (uaLength - MIN_UA_LENGTH) / (MAX_UA_LENGTH - MIN_UA_LENGTH);
  // Forearm parameters
  faLByte = (byte) 255 * (faLength - MIN_FA_LENGTH) / (MAX_FA_LENGTH - MIN_FA_LENGTH);
  // Shoulder Z position
  shZByte = (byte) 255 * (shPosZ - MIN_SHLDR_Z_POS) / (MAX_SHLDR_Z_POS - MIN_SHLDR_Z_POS);
}

void updateHumanLimbDynParamBytes()
{
  // Upper arm parameters
  uaWByte = (byte) 255 * (uaWeight - MIN_UA_WEIGHT) / (MAX_UA_WEIGHT - MIN_UA_WEIGHT);
  // Forearm parameters
  faWByte = (byte) 255 * (faWeight - MIN_FA_WEIGHT) / (MAX_FA_WEIGHT - MIN_FA_WEIGHT);
}

// void setHumanLimbParams(uint8_t uaL, uint8_t faL, uint8_t uaW, uint8_t faW, int8_t shZ)
// {
//   // Upper arm parameters
//   uaLByte = uaL;
//   uaLength = MIN_UA_LENGTH + (MAX_UA_LENGTH - MIN_UA_LENGTH) * (uaLByte / 255.0);
//   uaWByte = uaW;
//   uaWeight = MIN_UA_WEIGHT + (MAX_UA_WEIGHT - MIN_UA_WEIGHT) * (uaWByte / 255.0);
//   // Forearm parameters
//   faLByte = faL;
//   faLength = MIN_FA_LENGTH + (MAX_FA_LENGTH - MIN_FA_LENGTH) * (uaLByte / 255.0);
//   faWByte = uaW;
//   faWeight = MIN_FA_WEIGHT + (MAX_FA_WEIGHT - MIN_FA_WEIGHT) * (uaWByte / 255.0);
//   // Shoulder Z position
//   shZByte = shZ;
//   shPosZ = MIN_SHLDR_Z_POS + (MAX_SHLDR_Z_POS - MIN_SHLDR_Z_POS) * (shZByte / 255.0);
// }

void updateEndpointPosition()
{
  // arm inverse kinematics
  float _t1 = DEG2RAD(theta1);
  float _t2 = DEG2RAD(theta2);
  float _t3 = DEG2RAD(theta3);
  float _temp = L1 * cos(_t2) + L2 * cos(_t2 + _t3);
  xEp = cos(_t1) * _temp;
  yEp = sin(_t1) * _temp;
  zEp = - L1 * sin(_t2) - L2 * sin(_t2 + _t3);
}

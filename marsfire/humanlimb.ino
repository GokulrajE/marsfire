/* Functions to handle human limb related computation for data communication 
 * and control for the MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */

void setHumanLimbParams(uint8_t uaL, uint8_t faL, uint8_t uaW, uint8_t faW, int8_t shZ)
{
  // Upper arm parameters
  uaLByte = uaL;
  uaLength = MIN_UA_LENGTH + (MAX_UA_LENGTH - MIN_UA_LENGTH) * (uaLByte / 255.0);
  uaWByte = uaW;
  uaWeight = MIN_UA_WEIGHT + (MAX_UA_WEIGHT - MIN_UA_WEIGHT) * (uaWByte / 255.0);
  // Forearm parameters
  faLByte = faL;
  faLength = MIN_FA_LENGTH + (MAX_FA_LENGTH - MIN_FA_LENGTH) * (uaLByte / 255.0);
  faWByte = uaW;
  faWeight = MIN_FA_WEIGHT + (MAX_FA_WEIGHT - MIN_FA_WEIGHT) * (uaWByte / 255.0);
  // Shoulder Z position
  shZByte = shZ;
  shPosZ = MIN_SHLDR_Z_POS + (MAX_SHLDR_Z_POS - MIN_SHLDR_Z_POS) * (shZByte / 255.0);
}
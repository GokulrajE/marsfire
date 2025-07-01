/* Functions to handle human limb related computation for data communication 
 * and control for the MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */

void setHumanLimbParams(uint8_t uaL, uint8_t faL, uint8_t uaW, uint8_t faW,
                        int8_t shX, int8_t shY, int8_t shZ)
{
  // Upper arm parameters
  uaLByte = uaL;
  uaLength = MIN_UA_LENGTH + (MAX_UA_LENGTH - MIN_UA_LENGTH) * uaLByte;
  uaWByte = uaW;
  uaWeigth = MIN_UA_WEIGTH + (MAX_UA_WEIGTH - MIN_UA_WEIGTH) * uaWByte;
  // Forearm parameters
  faLByte = faL;
  faLength = MIN_FA_LENGTH + (MAX_FA_LENGTH - MIN_FA_LENGTH) * uaLByte;
  faWByte = uaW;
  faWeigth = MIN_FA_WEIGTH + (MAX_FA_WEIGTH - MIN_FA_WEIGTH) * uaWByte;
  // Shoulder position parameters
  shXByte = shX;
  shPosX = MIN_SHLDR_X_POS + (MAX_SHLDR_X_POS - MIN_SHLDR_X_POS) * shXByte;
  shYByte = shY;
  shPosY = MIN_SHLDR_Y_POS + (MAX_SHLDR_Y_POS - MIN_SHLDR_Y_POS) * shYByte;
  shZByte = shZ;
  shPosZ = MIN_SHLDR_Z_POS + (MAX_SHLDR_Z_POS - MIN_SHLDR_Z_POS) * shYByte;
}
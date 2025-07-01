/* Functions to implement the different controllers for the 
 *  MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */


// Changing control types must always be done by calling this function.
// This will ensure all the control related variables are set correctly.
void setControlType(byte ctype) {
  // Check if there is a change in control mode.
  if (ctrlType != ctype) {
    // Change control mode.
    ctrlType = ctype;
    // Initial control bound is set to zero.
    // ctrlBound = 0.0;
    // Set control direction.
    // ctrlDir = 0;
    // Set the target depending on the control mode.
    // if (ctrlType == TORQUE || ctrlType == TORQUELINEAR) {
    //   target = 0.0;
    // } else {
    //   target = INVALID_TARGET;
    //   // Set the contoller gains for the appropriate mechanism.
    //   setControlParamForMech();
    // }
    // Set control hold.
    // setControlHold(CONTROL_FREE);
    // Set desired target.
    // desired.add(INVALID_TARGET);
    // Reset object params
    // objPos = 0;
    // objDelPos = -1;    
    // Reset position controller error buffers.
    // err.add(0.0);
    // errdiff.add(0.0);
    // errsum.add(0.0);
  }
}
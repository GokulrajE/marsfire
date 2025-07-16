/* Functions to implement the different controllers for the 
 *  MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */


// Changing control types must always be done by calling this function.
// This will ensure all the control related variables are set correctly.
void setControlType(byte ctype) {
  // Setting the control type is dependent on what other information has 
  // been set in the firmware.
  switch (ctype) {
    case NONE:
    case POSITION:
      // Nothing needs to be set.
      ctrlType = ctype;
      target = 0.0;
      desired.add(INVALID_TARGET);
      return;
    case TORQUE:
      // Upper-limb kinematic and dynamic parameters need to set.
      if ((limbKinParam == NOLIMBKINPARAM) || (limbDynParam == NOLIMBDYNPARAM)) {
        ctrlType = NONE;
      } else {
        ctrlType = ctype;
        target = 0.0;
        desired.add(0.0);
      }
      return;
  }
}

void updateControlLaw() {
  // float _currI = 0.0;
  // float _currPWM = 0.0;
  // float _prevPWM = control.val(0);
  // bool _motorEnabled = true;
  // // Else we need to take the appropriate action.
  // switch (ctrlType) {
  //   case NONE:
  //     // Check control can be disabled.
  //     if (_prevPWM == 0) {
  //       digitalWrite(ENABLE, LOW);
  //       _motorEnabled = false;
  //     }
  //     break;
  //   case POSITION:
  //     // Position control.
  //     // Update the current desired position.
  //     desired.add(updateTargetPosition());
  //     _currI = controlPosition();
  //     _currPWM = boundPositionControl(convertCurrentToPWM(_currI));
  //     break;
  //   case POSITIONAAN:
  //     // // Position control.
  //     // _currI = controlPositionAAN();
  //     // _currPWM = boundPositionControl(convertCurrentToPWM(_currI));
  //     break;
  //   case TORQUE:
  //     // // Feedfoward torque control.
  //     // _currI = target.val(0) / MECHANICAL_CONST;
  //     // _currPWM = convertCurrentToPWM(_currI);
  //     break;
  //   case RESIST:
  //     // PD resistance control
  //     // desTorq = -(kp * (_ang - neutral_ang) + kd * 0.02 * _ang - kd * 0.02 * prev_ang);
  //     // cur = desTorq / mechnicalConstant;
  //     // __currpwm = constrain(map(abs(cur), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
  //     // prev_ang = _ang;
  //     break;
  // }
  // // Limit the rate of change of PWM
  // _currPWM = rateLimitValue(_currPWM, _prevPWM, MAXDELPWM);
  // // Clip PWM value
  // _currPWM = min(MAXPWM, max(-MAXPWM, _currPWM));
  // // Send PWM value to motor controller & update control.
  // if (_motorEnabled) {
  //   sendPWMToMotor(_currPWM);
  // }
  // control.add(_currPWM);
}



// float updateTargetPosition() {
//   if (target == INVALID_TARGET) return actual.val(0);
//   float _t = runTime.num / 1000.0f;
//   return strtPos + (target - strtPos) * mjt((_t - initTime) / reachDur);
// }
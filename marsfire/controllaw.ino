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
      target = INVALID_TARGET;
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
  // float _currPWM = 0.0;
  // float _currError = 0.0;
  float _currI = 0.0;
  float _currPWM = 0.0;
  float _prevPWM = control.val(0);
  bool _motorEnabled = true;
  float _alpha;
  // float ffCurr = 0.0;
  // float ffPWM = 0.0;
  // float _delPWMSign;
  // If control is NONE. Switch off control and move on.
  // if (ctrlType == NONE) {
  //   // Switch off controller.
  //   digitalWrite(ENABLE, LOW);
  //   control.add(0.0);
  //   return;
  // }
  // Else we need to take the appropriate action.
  switch (ctrlType) {
    case NONE:
      // Check control can be disabled.
      if (_prevPWM == 0) {
        digitalWrite(MOTOR_ENABLE, LOW);
        _motorEnabled = false;
      }
      // Keep the control buffers clean.
      err = 0.0;
      errdiff = 0.0;
      errsum = 0.0;
      control.add(0.0);
      break;
    case POSITION:
      // Update desired position
      desired.add(getDesiredValue());
      // Position control.
      _currI = limbControlScale * controlPosition();
      // Add the MARS gravity compensation torque
      marsGCTorque = MARS_GRAV_COMP_ADJUST * limbControlScale * getMarsGravityCompensationTorque() / MOTOR_T2I;
      _currI += marsGCTorque;
      _currPWM = boundPositionControl(convertCurrentToPWM(_currI));
      break;
    // case POSITIONAAN:
    //   // Check if its an invalid target
    //   desired.add(getAANDesiredTrajectory());
    //   // Position control.
    //   _currI = controlPositionAAN();
    //   _currPWM = boundPositionControl(convertCurrentToPWM(_currI));
    //   break;
    // case TORQUE:
    //   // Feedfoward torque control.
    //   _currI = target / MECHANICAL_CONST;
    //   _currPWM = convertCurrentToPWM(_currI);
    //   break;
    // case RESIST:
      // PD resistance control
      // desTorq = -(kp * (_ang - neutral_ang) + kd * 0.02 * _ang - kd * 0.02 * prev_ang);
      // cur = desTorq / mechnicalConstant;
      // __currpwm = constrain(map(abs(cur), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
      // prev_ang = _ang;
      // break;
  }
  // Limit the rate of change of PWM
  _currPWM = rateLimitValue(_currPWM, _prevPWM, MAXDELPWM);
  // Clip PWM value
  _currPWM = min(MAXPWM, max(-MAXPWM, _currPWM));
  // Send PWM value to motor controller & update control.
  if (_motorEnabled) {
    sendPWMToMotor(_currPWM);
  }
  control.add(_currPWM);
}

// Position controller
float controlPosition() {
  float _currang = actual.val(0);
  float _currtgt = desired.val(0);
  float _prevang = actual.val(1);
  float _prevtgt = desired.val(1);
  float _currp, _currd, _curri;
  float _currerr, _preverr;
  float _errsum = errsum;
  
  // Check if position control is disabled, or we should have valid 
  // current and previous desired positions. 
  if ((_currtgt == INVALID_TARGET) || (_prevtgt == INVALID_TARGET)) {
    err = 0.0;
    errdiff = 0.0;
    errsum = 0.0;
    return 0.0;
  }

  // Update error related information.
  // Current error
  _currerr = _currtgt - _currang;
  // Ignore small errors.
  _currerr = (abs((_currerr)) <= POS_CTRL_DBAND) ? 0.0 : _currerr;
  // Proportional control term.
  _currp = pcKp * (_currerr);

  // Previous error
  _preverr = (_prevtgt != INVALID_TARGET) ? _prevtgt - _prevang : _currerr;
  // Derivate control term.
  _currd = pcKd * (_currerr - _preverr);

  // Error sum.
  _errsum = 0.9999 * _errsum + _currerr;
  float _intlim = INTEGRATOR_LIMIT / pcKi;
  _errsum = min(_intlim, max(-_intlim, _errsum));
  // Integral control term.
  _curri = pcKi * _errsum;

  // Log error information
  err = _currp;
  errdiff = _currd;
  errsum = _curri;

  return _currp + _currd + _curri;
}

// Bound the position control output.
float boundPositionControl(float pwm_value) {
  if (pwm_value > MAXPWM) {
    return MAXPWM;
  } else if (pwm_value < -MAXPWM) {
    return -MAXPWM;
  }
  return pwm_value;
}

// Rate limit a variable.
float rateLimitValue(float curr, float prev, float rlim) {
  float _del = curr - prev;
  if (_del >= rlim) {
    return prev + rlim;
  } else if (_del <= -rlim) {
    return prev - rlim;
  }
  return curr;
}

// Convert current to PWM
float convertCurrentToPWM(float current) {
  float _sign = current >= 0 ? +1 : -1;
  return _sign * map(abs(current), 0, MAX_CURRENT, MINPWM, MAXPWM);
}

void sendPWMToMotor(float pwm) {
  if (pwm > 0) {
    // Move counter clockwise
    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, min(MAXPWM, max(pwm, MINPWM)));
    // return min(MAXPWM, max(pwm, MINPWM));
  } else {
    // Move counter clockwise
    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, min(MAXPWM, max(-pwm, MINPWM)));
    // return -min(MAXPWM, max(-pwm, MINPWM));
  }
}

byte limitPWM(float p) {
  if (p < 0) {
    p = -p;
  }
  if (p != 0)
    p = max(0.1, min(0.9, p));
  else
    p = 0.1;
  return (byte)(p * 255);
}

// Minimum jerk trajectory function
float mjt(float t) {
  t = t > 1 ? 1.0 : t;
  t = t < 0 ? 0.0 : t;
  return 6.0 * pow(t, 5) - 15.0 * pow(t, 4) + 10 * pow(t, 3);
}

// Compute the desired target value.
float getDesiredValue() {
  if (target == INVALID_TARGET) return actual.val(0);
  float _t = runTime.num / 1000.0f;
  return strtPos + (target - strtPos) * mjt((_t - initTime) / tgtDur);
}

// float updateTargetPosition() {
//   if (target == INVALID_TARGET) return actual.val(0);
//   float _t = runTime.num / 1000.0f;
//   return strtPos + (target - strtPos) * mjt((_t - initTime) / reachDur);
// }
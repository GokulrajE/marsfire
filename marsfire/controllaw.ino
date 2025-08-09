/* Functions to implement the different controllers for the 
 *  MARS robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 01 July 2025
 */


// Changing control types must always be done by calling this function.
// This will ensure all the control related variables are set correctly.
// This function will allow us to set a new control type only from the NONE
// control type. It does not allow transition between control modes.
bool setControlType(byte ctype) {
  // Reset beta and transition torque.
  beta = 0;
  transitionTorque = 0;
  // We want to change the current control type to NONE. No problem. Just
  // change it and leave.
  if (ctype == NONE) {
    ctrlType = NONE;
    target = INVALID_TARGET;
    desired.add(INVALID_TARGET);
    return true;
  }
  // We want to set some other control mode. This can be done only if the 
  // current control mode is NONE.
  // Leave if the current control type is not NONE.
  if (ctrlType != NONE) return false;
  // Alright, the current control model is NONE.
  switch (ctype) {
    case POSITION:
    case TORQUE:
      // Nothing needs to be set.
      ctrlType = ctype;
      break;
    case AWS:
      // Upper-limb kinematic and dynamic parameters need to set.
      if ((limbKinParam == NOLIMBKINPARAM) || (limbDynParam == NOLIMBDYNPARAM)) return false;
      ctrlType = ctype;
      break;
  }
  target = INVALID_TARGET;
  desired.add(INVALID_TARGET);
  return true;
}

// Transition control between POSITION and AWS. This is necessary for helping the 
// subject relax when in the rest state.
bool transitionControl(byte* payload, int strtInx) {
  // First byte in the payload is the new control type.
  // If this matches the current control type, then do nothing.
  if (ctrlType == payload[strtInx]) {
    return false;
  }
  
  // Its different.
  byte _ctrlType = payload[strtInx];

  // Read the target information.
  parseTargetDetails(serReader.payload, strtInx + 1, tempArray);
  // Check that the target is within the appropriate limits.
  // tempArray[2] has the target value.
  if ((_ctrlType == POSITION) && 
      (isWithinRange(tempArray[2], POSITION_TARGET_MIN, POSITION_TARGET_MAX) == false)) return false;
  if ((_ctrlType == AWS) &&
      (isWithinRange(tempArray[2], AWS_TARGET_MIN, AWS_TARGET_MAX) == false)) return false;
  
  // Set control type
  ctrlType = _ctrlType;
  
  // Set target.
  strtPos = ctrlType == AWS ? 1.0 : actual.val(0);
  strtTime = tempArray[1];
  target = tempArray[2];
  tgtDur = tempArray[3];

  // Initial time.
  initTime = runTime.num / 1000.0f + strtTime;
  
  // Set the current target set time.
  targetSetTime = runTime.num;

  // Record the transition torque.
  beta = 1.0;
  transitionTorque = torque;

  SerialUSB.print("Transition Ctrl: ");
  SerialUSB.print(ctrlType);
  SerialUSB.print(", ");
  SerialUSB.print(strtPos);
  SerialUSB.print(", ");
  SerialUSB.print(strtTime);
  SerialUSB.print(", ");
  SerialUSB.print(target);
  SerialUSB.print(", ");
  SerialUSB.print(tgtDur);
  SerialUSB.print(", ");
  SerialUSB.print(beta);
  SerialUSB.print(", ");
  SerialUSB.print(transitionTorque);
  SerialUSB.print("\n");

  return true;
}

void updateControlLaw() {
  float _currI = 0.0;
  float _currPWM = 0.0;
  float _prevPWM = control.val(0);
  bool _motorEnabled = true;
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
      desired.add(getDesiredPositionValue());
      // MARS gravity compensation torque
      marsGCTorque = MARS_GRAV_COMP_ADJUST * limbControlScale * getMarsGravityCompensationTorque() / MOTOR_T2I;
      _currI = marsGCTorque;
      // Transition torque weightage.
      beta = beta < 0.001 ? 0.0 : beta * AWS_TRANS_FACTOR;
      // Position control.
      _currI += beta * ( - MARS_GRAV_COMP_ADJUST * transitionTorque / MOTOR_T2I) + (1 - beta) * limbControlScale * controlPosition();
      _currPWM = convertCurrentToPWM(_currI);
      break;
    case TORQUE:
    case AWS:
      // Update desired position based on the control type.
      float _torqtgt;
      if (ctrlType == TORQUE) {
        _torqtgt = getDesiredTorqueValue();
      } else {
        // Get the weight support target.
        float _awstgt = getDesiredAWSTarget();
        // Compute the torque target.
        beta = beta < 0.001 ? 0.0 : beta * AWS_TRANS_FACTOR;
        _torqtgt = beta * transitionTorque + (1 - beta) * AWS_SCALE_FACTOR * _awstgt * hLimbTorque;
      }
      // float _torqtgt = ctrlType == TORQUE ? getDesiredTorqueValue() : getDesiredAWSTarget();
      // Scale target down based on moment arm and robot's flexion angle.
      float _scaleMA = SIGMOID((momentArm - TORQTGT_SIG_MA_MID) / TORQTGT_SIG_MA_SPRD);
      float _scaleShFlex = SIGMOID((theta1 - TORQTGT_SIG_FLX_MID) / TORQTGT_SIG_FLX_SPRD);
      desired.add(_scaleShFlex * _scaleMA * _torqtgt);
      // Add the MARS gravity compensation torque
      marsGCTorque = MARS_GRAV_COMP_ADJUST * limbControlScale * getMarsGravityCompensationTorque() / MOTOR_T2I;
      _currI = marsGCTorque;
      // Feedforward torque.
      _currI += - 0.5 * limbControlScale * desired.val(0);
      // Torque PD control.
      _currI += - limbControlScale * controlTorque();
      _currPWM = convertCurrentToPWM(_currI);
      break;
  }
  // Dampen control.
  _currPWM = dampenControlForSafety(_currPWM, ctrlType);
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
  float _currerr, _preverr, _currderr;
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
  _currderr = _currerr - _preverr;
  _currd = pcKd * _currderr;

  // Error sum.
  if (pcKi == 0) _curri = 0;
  else {
    _errsum = 0.9999 * _errsum + _currerr;
    float _intlim = INTEGRATOR_LIMIT / pcKi;
    _errsum = min(_intlim, max(-_intlim, _errsum));
    // Integral control term.
    _curri = pcKi * _errsum;
  }

  // Log error information
  err = _currp;
  errdiff = _currd;
  errsum = _currp + _currd; //_curri;

  return _currp + _currd + _curri;
}

float controlTorque() {
  float _currtorq = torque;
  float _currtgt = desired.val(0);
  float _prevtorq = torquePrev;
  float _prevtgt = desired.val(1);
  float _currp, _currd, _curri;
  float _currerr, _preverr;
  float _errsum = errsum;
  
  // // Check if position control is disabled, or we should have valid 
  // // current and previous desired positions. 
  if ((_currtgt == INVALID_TARGET) || (_prevtgt == INVALID_TARGET)) {
    err = 0.0;
    errdiff = 0.0;
    errsum = 0.0;
    return 0.0;
  }

  // Update error related information.
  // Current error
  _currerr = _currtgt - _currtorq;
  // Ignore small errors.
  _currerr = (abs((_currerr)) <= TORQUE_CTRL_DBAND) ? 0.0 : _currerr;
  // Proportional control term.
  _currp = tcKp * (_currerr);

  // Previous error
  _preverr = (_prevtgt != INVALID_TARGET) ? _prevtgt - _prevtorq : _currerr;
  // Derivate control term.
  _currd = tcKd * (_currerr - _preverr);

  // Error sum.
  if (tcKi == 0) _curri = 0;
  else {
    _errsum = 0.9999 * _errsum + _currerr;
    float _intlim = INTEGRATOR_LIMIT / tcKi;
    _errsum = min(_intlim, max(-_intlim, _errsum));
    // Integral control term.
    _curri = tcKi * _errsum;
  }

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

// Dampen the control output if the speed is too fast.
float dampenControlForSafety(float currPWM, byte cType) {
  float _bDamp = 0.0;
  float _sign = omega1 > 0 ? +1 : -1;
  float _vel;
  switch(ctrlType) {
    case NONE:
      _bDamp = omega1 > SAFETY_DAMP_VEL_TH ? SAFETY_DAMP_VALUE : 0.0;
      _vel = abs(omega1 - SAFETY_DAMP_VEL_TH) / 5.0;
      break;
    case POSITION:
    case TORQUE:
      _bDamp = abs(omega1) > SAFETY_DAMP_VEL_TH ? SAFETY_DAMP_VALUE : 0.0;
      _vel = abs(omega1 - SAFETY_DAMP_VEL_TH) / 5.0;
      break;
    case AWS:
      _bDamp = 2.0 * SAFETY_DAMP_VALUE;
      _vel = abs(omega1) / 5.0;
      break;
  }
  currPWM += - limbControlScale * _bDamp * _sign * _vel * _vel;
  return currPWM;
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

// Compute the desired position target value.
float getDesiredPositionValue() {
  if (target == INVALID_TARGET) return actual.val(0);
  float _t = runTime.num / 1000.0f;
  float _tgt = strtPos + (target - strtPos) * mjt((_t - initTime) / tgtDur);
  return min(POSITION_TARGET_MAX, max(POSITION_TARGET_MIN, _tgt));
}

// Compute the desired torque target value.
float getDesiredTorqueValue() {
  if (target == INVALID_TARGET) return 0.0;
  float _t = runTime.num / 1000.0f;
  float _tgt = strtPos + (target - strtPos) * mjt((_t - initTime) / tgtDur);
  return min(TORQUE_TARGET_MAX, max(TORQUE_TARGET_MIN, _tgt));
}

// Compute the desired AWS target value.
float getDesiredAWSTarget() {
  float _t = runTime.num / 1000.0f;
  float _tgt = strtPos + (target - strtPos) * mjt((_t - initTime) / tgtDur);
  return min(AWS_TARGET_MAX, max(AWS_TARGET_MIN, _tgt));
}

// Desired torque scaler.


// float updateTargetPosition() {
//   if (target == INVALID_TARGET) return actual.val(0);
//   float _t = runTime.num / 1000.0f;
//   return strtPos + (target - strtPos) * mjt((_t - initTime) / reachDur);
// }
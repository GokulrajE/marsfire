
/*
 * Check if there is heartbeat.
 */
void checkHeartbeat() {
  // Check if a heartbeat was recently received.
  if (0.001 * (millis() - lastRxdHeartbeat) < MAX_HBEAT_INTERVAL) {
    // Everything is good. No heart beat related error.
    deviceError.num &= ~NOHEARTBEAT;
  } else {
    // No heartbeat received.
    // Setting error flag.
    deviceError.num |= NOHEARTBEAT;
  }
}

/*
 * Check encoder-imu mismatch errors
 */
void checkEncoderIMUMismatch() {
  if ((calib == NOCALIB) || (abs(theta1 - imuTheta1) < IMU_MISMATCH_ERROR)) {
    deviceError.num &= ~ANG1MISMATCHERR;
    mismatchCount = 0;
  } else {
    if (mismatchCount * delTime > 0.5) {
      deviceError.num |= ANG1MISMATCHERR;
    } else {
      mismatchCount++;
    }
  }
}

/*
 * Check angle jump errors
 */
void checkEncoderJump() {
  // Angle sensor 1 jump
  if ((calib == NOCALIB) || (abs(theta1 - theta1Prev) < ANG_JUMP_ERROR)) {
    deviceError.num &= ~ANG1JUMPERR;
  } else {
    deviceError.num |= ANG1JUMPERR;
  }
}

/*
 * Handles errors
 */
void handleErrors() {
  if (deviceError.num != 0) {
    setControlType(NONE);
  }
  // Check if its angle related error.
  if ((deviceError.num == ANG1MISMATCHERR)
      || (deviceError.num == ANG234MISMATCHERR)
      || (deviceError.num == ANG1JUMPERR)
      || (deviceError.num == ANG234JUMPERR)) {
    setLimb(NOLIMB);
    calib = NOCALIB;
  }
}

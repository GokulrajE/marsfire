
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
 * Handles errors
 */
void handleErrors() {
  if (deviceError.num != 0) {
    // #if SERIALUSB_DEBUG
    //   SerialUSB.print("Error occured: ");
    //   SerialUSB.print(deviceError.num);
    //   SerialUSB.print("\n");
    // #endif
    setControlType(NONE);
  }
}

/*
 * Update the safety timers.
 */
void updateSafetyTimers() {
  // Target set blackout timer.
  if (safetyTimerTargetSetBlackout >= 0) safetyTimerTargetSetBlackout++;
}
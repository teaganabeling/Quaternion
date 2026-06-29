
bool AccelBma400::readData()
{
  // Get the hardware interrupt status
  if ( digitalRead(ACCEL_INT1_PIN) )
  {
    tsd.hwFlags |= (1UL << ACCEL_INT1_TYPE);      // Set the appropriate HW Int Flag
    this->processInterrupt();
  }
  else
  {
    tsd.hwFlags &= ~(1UL << ACCEL_INT1_TYPE);     // Clear the appropriate HW Int Flag
  }

  int8_t result = -128;
  {
    auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
    if ( ! p_sem ) { return false; }
    result = _IMU.getSensorData(true);
  }

  if ( result == BMA400_OK )
  {
    if ( ! tsd.takeSemaphore(__PRETTY_FUNCTION__) ) { return false; }

    tsd.X = _IMU.data.X << 4; // Shift up the raw data to scale match with different bit resolutions
    tsd.Y = _IMU.data.Y << 4; // ONE_G = 16384 with 2G sensitivity
    tsd.Z = _IMU.data.Z << 4;
    tsd.sensorTimeMillis = _IMU.data.sensorTimeMillis;

    if ( tsd.X == -1 && tsd.Y == -1 && tsd.Z == -1 )
    {
      tsd.swFlags &= ~(1UL << VALID_ACCEL_SW_FLAG);     // Clear the Valid Accel Data Flag
    }
    else
    {
      tsd.swFlags |= (1UL << VALID_ACCEL_SW_FLAG);      // Set the Valid Accel Data Flag

      // Only use the 14 most significant bits
      // tsd.X >>= 2; tsd.Y >>= 2; tsd.Z >>= 2;
    }

    // cfg.accelTareX = 0 - tsd.X;     // tsd.X + cfg.accelTareX = 0
    // cfg.accelTareY = 0 - tsd.Y;     // tsd.Y + cfg.accelTareY = 0
    // cfg.accelTareZ = 16384 - tsd.Z; // tsd.Z + cfg.accelTareZ = 16384

    float tpx = (float) cfg.accelTareX / ONE_G;
    float tpy = (float) cfg.accelTareY / ONE_G;
    float tpz = (float) cfg.accelTareZ / ONE_G;
    float tmod = sqrt(tpx*tpx + tpy*tpy + tpz*tpz);

    float gpx = (float) tsd.X / ONE_G;
    float gpy = (float) tsd.Y / ONE_G;
    float gpz = (float) tsd.Z / ONE_G;
    float gmod = sqrt(gpx*gpx + gpy*gpy + gpz*gpz);

    // Current accel in g's
    Vec3 a = { gpx, gpy, gpz };

    // Only trust accel when magnitude is near 1g (tune thresholds)
    if (gmod > 0.8f && gmod < 1.2f)
    {
    Vec3 g_unit  = a / gmod;

    // Calibration gravity direction in sensor frame (unit)
    Vec3 g0_raw  = {
        (float)cfg.accelTareX / ONE_G,
        (float)cfg.accelTareY / ONE_G,
        (float)cfg.accelTareZ / ONE_G
    };
    float g0_mod = norm(g0_raw);

    if (g0_mod > 0.5f)  // sanity check; should be ~1
    {
        Vec3 g0_unit = g0_raw / g0_mod;

        Quat q_tilt = quatFrom2UnitVectors(g0_unit, g_unit);

        tsd.zAngle = quatAngleDeg(q_tilt);

        float roll_deg = 0.0f, pitch_deg = 0.0f;
        quatToRollPitchDeg(q_tilt, &roll_deg, &pitch_deg);
        tsd.zRoll  = roll_deg;
        tsd.zPitch = pitch_deg;
    }
    // else: keep last good angles, or mark invalid
    }
    else
    {
    // In motion: either keep last, or mark invalid
    }



    // Check to see if we are lingering in the alert zone
    _zoneCount = orientationGood() ? 0 : _zoneCount + 1;
    if ( _zoneCount > cfg.alertHoldoffCount ) {
      tsd.swFlags |= (1UL << ALERT_ZONE_SW_FLAG);      // Set the Alert Zone SW Flag
    } else {
      tsd.swFlags &= ~(1UL << ALERT_ZONE_SW_FLAG);     // Clear the Alert Zone SW Flag
    }
    tsd.zoneCount = (_zoneCount > 255) ? 255 : (uint8_t)(_zoneCount & 0xFF);

    // Check and report if any FIFO flags occurred since last readData
    if ( _fifoOverflow > 0 ) {
      tsd.fifoFlags |= (1UL << OVERFLOW_FIFO_FLAG);          // Set the Overflow FIFO Flag
      _fifoOverflow = 0;                                     // Clear the counter
    } else {
      tsd.fifoFlags &= ~(1UL << OVERFLOW_FIFO_FLAG);         // Clear the Overflow FIFO Flag
    }
    if ( _fifoMismatch > 0 ) {
      tsd.fifoFlags |= (1UL << MISMATCH_FIFO_FLAG);          // Set the Mismatch FIFO Flag
      _fifoMismatch = 0;                                     // Clear the counter
    } else {
      tsd.fifoFlags &= ~(1UL << MISMATCH_FIFO_FLAG);         // Clear the Mismatch FIFO Flag
    }
    if ( _fifoPersistOverflow > 0 ) {
      tsd.fifoFlags |= (1UL << PERSIST_OVERFLOW_FIFO_FLAG);  // Set the Persist Overflow FIFO Flag
      _fifoPersistOverflow = 0;                              // Clear the counter
    } else {
      tsd.fifoFlags &= ~(1UL << PERSIST_OVERFLOW_FIFO_FLAG); // Clear the Persist Overflow FIFO Flag
    }
    if ( _breathRateErrors > 0 ) {
      tsd.fifoFlags |= (1UL << BREATHRATE_ERROR_FIFO_FLAG);  // Set the Breathrate Error FIFO Flag
      _breathRateErrors = 0;                                 // Clear the counter
    } else {
      tsd.fifoFlags &= ~(1UL << BREATHRATE_ERROR_FIFO_FLAG); // Clear the Breathrate Error FIFO Flag
    }
  }
  else
  {
    if ( ! tsd.takeSemaphore(__PRETTY_FUNCTION__) ) { return false; }
    tsd.X = -1;
    tsd.Y = -1;
    tsd.Z = -1;

    tsd.swFlags &= ~(1UL << VALID_ACCEL_SW_FLAG);       // Clear the Valid Accel Data Flag
  }
  tsd.fifoFlags &= ~(1UL << FFT_VALID_FIFO_FLAG);       // Clear the FFT Valid FIFO flag
  tsd.giveSemaphore();
  return this->doFFT();
}
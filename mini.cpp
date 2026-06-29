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

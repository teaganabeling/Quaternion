// WhisperPT_Device
// accel_bma400.cpp

#include "accel_bma400.h"
#include "time_series_data.h"
#include "hardware_config.h"
#include "software_config.h"
#include "i2c_bus.h"
#include "fsm_states.h"
#include <nrf_gpio.h>
#include <task.h>
#include <arm_math.h>
#include <arm_const_structs.h>

extern xTaskHandle g_pCollectTSDTaskHandle;
extern bool g_debug_int;
extern bool g_debug_fifo;
extern bool g_debug_fft;

AccelBma400 imu_bma400;
ZAngleTracking zat;
uint8_t g_zatQueueStorageArea[ ZAT_QUEUE_LENGTH * ZAT_QUEUE_ITEM_SIZE ];
TimeSeriesDataQueue g_zatQueue(ZAT_QUEUE_LENGTH, ZAT_QUEUE_ITEM_SIZE, &g_zatQueueStorageArea[0]);

AccelBma400::AccelBma400(uint8_t addressI2c) :
  _IMU(),
  _addressI2c(addressI2c)
{
  AccelHardware::add(this);
}

bool AccelBma400::isDetected()
{
  return this->checkDevice();
}

bool AccelBma400::isFifo()
{
  return true;
}

bool AccelBma400::clearInterrupt()
{
  uint16_t interruptStatus = 0;
  auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
  if ( ! p_sem ) { return false; }

  int8_t result = _IMU.getInterruptStatus(&interruptStatus);
  return ( result == BMA400_OK );
}

void AccelBma400::processInterrupt()
{
  bool pinStatus = digitalRead(ACCEL_INT1_PIN);
  // Get the software interrupt status
  uint16_t interruptStatus = 0;
  int8_t result = -128;
  {
    auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
    if ( ! p_sem ) { return; }
    result = _IMU.getInterruptStatus(&interruptStatus);
  }

  if ( result == BMA400_OK )
  {
    if (g_debug_int) {
      _console->printf("%s: 0x%04x, %d, %d\n", __PRETTY_FUNCTION__, interruptStatus, pinStatus, digitalRead(ACCEL_INT1_PIN));
    }
    // Check if movement has occurred
    if(interruptStatus & BMA400_ASSERTED_GEN1_INT)
    {
      tsd.swFlags |= (1UL << ACCEL_INT1_SW_FLAG);      // Set the Accel SW Int 1 Flag
      tsd.motionCount++;
      _console->println("Motion Detected");
    }
    else
    {
      tsd.swFlags &= ~(1UL << ACCEL_INT1_SW_FLAG);     // Clear the Accel SW Int 1 Flag
    }

    // Check if a FIFO full condition has occurred
    if(interruptStatus & BMA400_ASSERTED_FIFO_FULL_INT)
    {
      _fifoOverflow++;
      uint16_t fifo_length = 0;
      result = -128;
      {
        auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
        if ( ! p_sem ) { return; }
        result = _IMU.getFIFOLength(&fifo_length);
        _IMU.flushFIFO();
        _IMU.getInterruptStatus(&interruptStatus);
      }
      // Clear FIFO Full and Watermark interrupt status bits in this context
      interruptStatus &= ~(BMA400_ASSERTED_FIFO_FULL_INT | BMA400_ASSERTED_FIFO_WM_INT);
      _console->printf("FIFO Overflow, flushed: %d samples\n", fifo_length);
    }

    // Check if this is the FIFO watermerk interrupt condition
    if(interruptStatus & BMA400_ASSERTED_FIFO_WM_INT)
    {
      this->readFifoData();
    }

  }

  //xTaskNotify (g_pCollectTSDTaskHandle,   1, eSetValueWithOverwrite);
}

void AccelBma400::readFifoData()
{
  // Get FIFO data from the sensor
  uint16_t samplesRead = ACCEL_DATA_NUM_SAMPLES;
  uint32_t zat_block_data[ZAT_BLOCK_DATA_LENGTH];
  int8_t result = -128;
  {
    auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
    if ( ! p_sem ) { return; }
#if ( ZAT_BLOCK_DATA_LENGTH > 1 )
    zat_block_data[1] = millis();
#endif

    result = _IMU.getSensorData(true);
    if ( result != BMA400_OK ) {
      _console->printf("FIFO: getSensorData failed: %d\n", result);
      return;
    }

    result = _IMU.getFIFOData(_fifoData, &samplesRead);
    if ( result != BMA400_OK ) {
      _console->printf("FIFO: getFIFOData failed: %d, %d\n", result, samplesRead);
      uint16_t fifo_length = 0;
      result = _IMU.getFIFOLength(&fifo_length);
      _console->printf("FIFO length: %d, %d\n", fifo_length, result);
      return;
    }

    // Clear the watermark interrupt
    uint16_t interruptStatus;
    _IMU.getInterruptStatus(&interruptStatus);

    // Add non-computed block framing data
    zat_block_data[0] = 0;
    //zat_block_data[0] |= (zSignLo & (0x1))         << 0;
    //zat_block_data[0] |= (zSignHi & (0x1))         << 1;
    zat_block_data[0] |= (tsd.breathRate & (0x3F))   << 2;
    zat_block_data[0] |= (tsd.state & (0xF))         << 8;
    zat_block_data[0] |= (tsd.alertLevel & (0x7))    << 12;
    //zat_block_data[0] |= (fifoError & (0x1))       << 15;
    zat_block_data[0] |= ((_IMU.data.Y >> 4) & 0xFF) << 16;
    zat_block_data[0] |= ((_IMU.data.X >> 4) & 0xFF) << 24;
#if ( ZAT_BLOCK_DATA_LENGTH > 2 )
    zat_block_data[2] = 0;
    zat_block_data[2] |= ((tsd.heapFree) & 0xFFFC)   << 0;
    zat_block_data[2] |= ((tsd.vBat) & 0xFFFF)       << 16;
#endif
#if ( ZAT_BLOCK_DATA_LENGTH > 3 )
    int8_t core_temp = NRF_TEMP->TEMP - 100;
    zat_block_data[3] = 0;
    zat_block_data[3] |= ((tsd.fifoFlags >> 1) & 0xF) << 0;
    zat_block_data[3] |= ((_IMU.data.X) & 0xF)        << 4;
    //zat_block_data[3] |= ((tsd.hearRate) & 0xFF)      << 8;
    zat_block_data[3] |= ((_IMU.data.Y) & 0xF)        << 8;
    zat_block_data[3] |= ((_IMU.data.Z) & 0xFFF)      << 12;
    zat_block_data[3] |= ((core_temp) & (0xFF))       << 24;
#endif
  }

  // samplesRead will be changed to the number of data frames actually
  // read from the FIFO buffer. Check whether it's equal to ACCEL_DATA_NUM_SAMPLES
  if(samplesRead != ACCEL_DATA_NUM_SAMPLES)
  {
    // Most likely didn't have enough data frames in FIFO buffer.
    // This can happen if control frames are inserted into the FIFO
    // buffer, which occurs when certain configuration changes occur
    _fifoMismatch++;
    zat_block_data[0] |= (1 << 15);  // Set fifoError bit in block framing data
    _console->print("Unexpected number of samples read from FIFO: ");
    _console->println(samplesRead);
  }

  // Check if we exceeded queue capacity for: zat.zAnglePack
  if ( (_accelSampleCount - _accelSamplePersisted) >= (ZAT_QUEUE_LENGTH * TSD_Z_ANGLE_BLOCK_SIZE) )
  {
    _fifoPersistOverflow++;
    zat_block_data[0] |= (1 << 15);  // Set fifoError bit in block framing data
    _console->println("FIFO Persist Overflow");
  }

  // Pre-calculate refrence vector
  float tpx = (float) cfg.accelTareX / ONE_G;
  float tpy = (float) cfg.accelTareY / ONE_G;
  float tpz = (float) cfg.accelTareZ / ONE_G;
  float tmod = sqrt(tpx*tpx + tpy*tpy + tpz*tpz);

  // Print out all acquired data
  uint32_t block_data_index = 0;
  for(uint16_t i = 0; i < samplesRead; i++)
  {

    //float32_t acc[3];
    //float32_t arg1 = 0;
    //arg1 += (float32_t) (acc[0]) * (float32_t) (acc[0]);
    //arg1 += (float32_t) (acc[1]) * (float32_t) (acc[1]);
    //arg1 += (float32_t) (acc[2]) * (float32_t) (acc[2]);
    //arm_sqrt_f32(arg1, &sqrt_result);
    //TiltAngleInRad = arm_cos_f32((acc[2]) / sqrt_result);

    float gpx = _fifoData[i].accelX;
    float gpy = _fifoData[i].accelY;
    float gpz = _fifoData[i].accelZ;
    float gmod = sqrt(gpx*gpx + gpy*gpy + gpz*gpz);
    float zCos = ( tpx*gpx + tpy*gpy + tpz*gpz ) / (tmod*gmod);
    float zAngle = acos( zCos ) * 4068/71;
    //_console->println(zAngle, 2);
    //_console->printf("%0.2f, %0.2f, %0.2f, %0.2f\n", zAngle, gpx, gpy, gpz);

    uint32_t zat_index = _accelSampleCount % TSD_Z_ANGLE_BLOCK_SIZE;
    zat.zAnglePack[zat_index] = zAngle * 65536 / 180; // 4068/71 (above) ensures this will always be less than 65536
    zat.zAnglePack[zat_index] &= ~(0x1); // Clear bit0 to create space to store frame data across the block
    _zAngleCacheBuffer[_accelSampleCount % ACCEL_DATA_CACHE_BUFFER_LENGTH] = zAngle; // Loops every 512 bytes _accelSampleCount & 0x1FF
    _zoneCount = orientationGood(zAngle) ? 0 : _zoneCount + 1;

    // Set the orientation bit twice in for every zat block
    if ( zat_index % (TSD_Z_ANGLE_BLOCK_SIZE / 2) == 0 ) {
      // WhisperPT - PAP models use X, EAR models use Z for sleeping side determination
      bool zSign = ( hwCfg.productID == 0x802A ) ? ( _fifoData[i].X >= 0 ) : ( _fifoData[i].Z >= 0 );
      zat.zAnglePack[zat_index / (TSD_Z_ANGLE_BLOCK_SIZE / 2)] |= zSign;
    }

    // Integrate the rest of the block data into zAnglePack
    const uint8_t bits_per_block_data = (sizeof(zat_block_data[0]) * 8);
    if ( zat_index >= (TSD_Z_ANGLE_BLOCK_SIZE - 1) ) {
      for(uint16_t j = 0; j < (TSD_Z_ANGLE_BLOCK_SIZE / bits_per_block_data); j++)
      {
        uint32_t bit_data = zat_block_data[block_data_index];
        uint8_t bit_offset = j * bits_per_block_data;
        if (g_debug_fifo) {
          _console->printf("Integrate ZAT block data: %d, at offset: %d\n", block_data_index, bit_offset);
        }
        for(uint16_t i = 0; i < bits_per_block_data; i++)
        {
          zat.zAnglePack[i+bit_offset] |= (bit_data & 0x1);
          bit_data >>= 1;
        }
        block_data_index++;
        if ( block_data_index >= ZAT_BLOCK_DATA_LENGTH ) {
          block_data_index = 0;
        }
      }
      zat.snapshotData();
      _accelSamplePersisted += TSD_Z_ANGLE_BLOCK_SIZE;
    }
    _accelSampleCount++;
  }

  uint32_t zat_index = _accelSampleCount % TSD_Z_ANGLE_BLOCK_SIZE;
  if (g_debug_fifo && zat_index) {
    _console->printf("FIFO Zero ZAT from index: %d\n", zat_index);
  }

  // Zero any leftover space in the current zAnglePack TSD block
  for (uint32_t i = zat_index; i <  TSD_Z_ANGLE_BLOCK_SIZE; i++)
  {
    zat.zAnglePack[i] = 0;
  }

  if (g_debug_fifo) {
    _console->printf("FIFO Sample: %d, Overflow: %d, Mismatch: %d, PersistOverflow: %d, ZoneCount: %d\n", _accelSampleCount, _fifoOverflow, _fifoMismatch, _fifoPersistOverflow, _zoneCount);
  }

  if ( _zoneCount > cfg.alertHoldoffCount ) {
    tsd.swFlags |= (1UL << ALERT_ZONE_SW_FLAG);      // Set the Alert Zone SW Flag
    xTaskNotify (g_pCollectTSDTaskHandle,   1, eSetValueWithOverwrite);
  } else {
    if ( tsd.swFlags & (1UL << ALERT_ZONE_SW_FLAG) ) {
      tsd.swFlags &= ~(1UL << ALERT_ZONE_SW_FLAG);     // Clear the Alert Zone SW Flag
      xTaskNotify (g_pCollectTSDTaskHandle,   1, eSetValueWithOverwrite);
    }
  }
}

bool AccelBma400::doFFT()
{
  if (g_debug_fft) { _console->println(__PRETTY_FUNCTION__); }
  // Prepare input buffer for FFT
  //uint16_t n = _accelSampleCount % ACCEL_DATA_CACHE_BUFFER_LENGTH;
  //memcpy( &_zAngleFftBuffer[0], &_zAngleCacheBuffer[n], ACCEL_DATA_CACHE_BUFFER_LENGTH-n );
  //memcpy( &_zAngleFftBuffer[ACCEL_DATA_CACHE_BUFFER_LENGTH-n], &_zAngleCacheBuffer[0], n );
  uint16_t ringBufferStart = _accelSampleCount % ACCEL_DATA_CACHE_BUFFER_LENGTH;
  uint32_t sensorTimeMillis = _IMU.data.sensorTimeMillis;
  for(uint16_t i = 0; i < ACCEL_DATA_CACHE_BUFFER_LENGTH; i++)
  {
    uint16_t bufferPosition = ringBufferStart + i;
    if ( bufferPosition >= ACCEL_DATA_CACHE_BUFFER_LENGTH ) {
      bufferPosition -=  ACCEL_DATA_CACHE_BUFFER_LENGTH;
    }
    _zAngleFftBuffer[i*2] = _zAngleCacheBuffer[bufferPosition];
    _zAngleFftBuffer[i*2+1] = 0.0;
  }

  if (g_debug_fft) {
    _console->println("zAngleFftBuffer:");
    for(uint16_t i = 0; i < ACCEL_DATA_FFT_BUFFER_LENGTH; i++)
    {
      _console->print(_zAngleFftBuffer[i]); _console->print(", ");
      if ( (i + 1) % 8 == 0 )
      {
        _console->println();
      }
      if ( (i + 1) % 256 == 0 )
      {
        _console->println();
      }
    }
    _console->println();
  }

  // Prep FFT config values, this only needs to be done once
  const uint32_t fftSize = ACCEL_DATA_CACHE_BUFFER_LENGTH;
  arm_cfft_instance_f32 varInstCfftF32;
  arm_status status = ARM_MATH_SUCCESS;
  status = arm_cfft_init_f32(&varInstCfftF32,fftSize);

  /* Process the data through the CFFT/CIFFT module */
  arm_cfft_f32(&varInstCfftF32, _zAngleFftBuffer, 0, 1);  // ifftFlag = 0, doBitReverse = 1

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  static float32_t fftOutput[fftSize];
  arm_cmplx_mag_f32(_zAngleFftBuffer, fftOutput, fftSize);

  // Initialize TSD values before continuing
  if ( ! tsd.takeSemaphore(__PRETTY_FUNCTION__) ) { return false; }
  tsd.accelSampleCount = _accelSampleCount;
  tsd.accelSamplePersisted = _accelSamplePersisted;
  tsd.sensorTimeMillis = sensorTimeMillis;
  tsd.numSamples = fftSize;
  tsd.odrSize = BMA400_ODR_25HZ;
  tsd.breathRateNonPeak = 0;
  tsd.dcOffset = 0;
  tsd.dcOffsetBinCount = 0;
  for(uint16_t i = 0; i < (sizeof(tsd.fftRank)/sizeof(tsd.fftRank[0])); i++)
  {
    tsd.fftRank[i].index = 0;
    tsd.fftRank[i].magnitude = 0;
  }

  // Remove initial spike due to DC Offset
  tsd.dcOffset = fftOutput[0];
  if (g_debug_fft) {
    _console->print(fftOutput[0]); _console->println(", DC Offset");
  }
  for(uint16_t i = 0; i < 10; i++)
  {
    float32_t prevOutput = fftOutput[i];
    if (g_debug_fft) {
      _console->printf("Remove DC Offset bin: %d, %0.2f\n", i, fftOutput[i]);
    }
    fftOutput[i] = 0;
    tsd.dcOffsetBinCount++;
    if ( fftOutput[i+1] > prevOutput ) { break; }
  }

  float32_t frequencyStep = 25.0/fftSize;
  float32_t maxValue;
  uint32_t maxIndex = 0;

  // Find breating rate
  arm_max_f32(fftOutput, fftSize/2, &maxValue, &maxIndex);
  uint16_t breathRate = maxIndex * frequencyStep * 60 + 0.5;
  uint8_t breathRank = 1;
  if (g_debug_fft) {
    _console->print("Peak Rate: "); _console->print(breathRate); _console->println(" bpm");
    _console->print(maxValue); _console->print(", "); _console->println(maxIndex * frequencyStep);
    _console->print("Frequency Step: "); _console->println(frequencyStep);
  }
  tsd.breathRate = (breathRate > 255) ? 255 : breathRate;
  tsd.breathRateNonPeak = 0;
/*
  if (g_debug_fft) {
    for(uint16_t i=0; i<fftSize/2; i++)
    {
      _console->print(fftOutput[i]); _console->print(", "); _console->println(i * frequencyStep);
    }
  }
*/
  if (g_debug_fft) {
    _console->printf("Top %d ...\n", (sizeof(tsd.fftRank)/sizeof(tsd.fftRank[0])));
  }
  for(uint16_t i = 0; i < (sizeof(tsd.fftRank)/sizeof(tsd.fftRank[0])); i++)
  {
    arm_max_f32(fftOutput, fftSize/2, &maxValue, &maxIndex);
    float32_t maxRate = maxIndex * frequencyStep * 60;
    if (g_debug_fft) {
      _console->printf("%d: %0.2f, %0.2f (%0.2f bpm)\n", i, maxValue, maxIndex * frequencyStep, maxRate);
    }
    tsd.fftRank[i].index = maxIndex;
    tsd.fftRank[i].magnitude = (maxValue > 65535) ? 65535 : maxValue;
    fftOutput[maxIndex] = 0;
    if ( (breathRate > 30) && (maxRate < 30) )
    {
      breathRate = maxRate + 0.5;
      tsd.breathRateNonPeak = breathRate;
      breathRank = i + 1;
      if (g_debug_fft) {
        _console->print("Non-Peak Breath Rate: "); _console->println(breathRate);
      }
    }
  }

  if (breathRate > 30)
  {
    _breathRateErrors++;
    if (g_debug_fft) {
      _console->print("Breath Rate Error: "); _console->print(breathRate);
      _console->print(", Rank: "); _console->println(breathRank);
    }
    breathRate = 0;
    breathRank = 0;
  }
  else
  {
    if (g_debug_fft) {
      _console->print("Breathing Rank: "); _console->println(breathRank);
      _console->print("Breathing Rate: "); _console->print(breathRate); _console->println(" breaths/min");
    }
  }
  if (g_debug_fft) { _console->println(); }
  tsd.breathRateErrors = _breathRateErrors;
  tsd.fifoFlags |= (1UL << FFT_VALID_FIFO_FLAG); // Set the FFT Valid FIFO flag
  tsd.giveSemaphore();
  return true;
}

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

    // Compute the angle in degrees between tare vector and current vector
    tsd.zAngle = acos( ( tpx*gpx + tpy*gpy + tpz*gpz ) / (tmod*gmod) ) * 4068/71;

    // Rotate the current vector into the head reference frame
//    float ax = g_rMatrixA11 * gpx + g_rMatrixA21  * gpx + g_rMatrixA31  * gpx;
//    float ay = g_rMatrixA12 * gpy + g_rMatrixA22  * gpy + g_rMatrixA32  * gpy;
//    float az = g_rMatrixA13 * gpz + g_rMatrixA23  * gpz + g_rMatrixA33  * gpz;

    // This should provide relative pitch and roll of the head from tare position
//    tsd.zRoll  = atan2(ay , az) * 4068/71;
//    tsd.zPitch = atan2((-ax) , sqrt(ay * ay + az * az)) * 4068/71;

    // This provides the absolute pitch and roll, (non compensated)
    tsd.zRoll  = atan2(gpy , gpz) * 4068/71;
    tsd.zPitch = atan2((-gpx) , sqrt(gpy * gpy + gpz * gpz)) * 4068/71;

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

bool AccelBma400::begin()
{
  auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
  if ( ! p_sem ) { return false; }

  int8_t result = _IMU.beginI2C(_addressI2c);

  if ( result == BMA400_OK ) {
    // Create the zAngleTracking Queue
    zat.beginQueue(&g_zatQueue);

    // The default ODR (output data rate) is 200Hz, which is too fast to read
    // reasonably. Here we reduce the ODR to 25Hz
    _IMU.setODR(BMA400_ODR_25HZ);

    // Sacrifice a bit of noise (0.74 -> 1.81) for 11uA (14.5 -> 3.5) of
    // power savings
    _IMU.setOSR(BMA400_ACCEL_OSR_SETTING_0);

    // Decreasing the range from the default of 4g to the min of 2g helps
    // capture more precise movement
    _IMU.setRange(BMA400_RANGE_2G);

    // Here we set the config parameters for the FIFO buffer. There are several
    // flags that we can set, which are listed below. Additionally, we can set
    // the watermark level, and choose where to route the interrupt conditions.
    // conf_regs flags:
    // BMA400_FIFO_AUTO_FLUSH   - Flush FIFO when power mode changes
    // BMA400_FIFO_STOP_ON_FULL - Stop storing data when FIFO is full
    // BMA400_FIFO_TIME_EN      - Log sensor time when FIFO is read out
    // BMA400_FIFO_DATA_SRC     - Store data from filter 2 instead of filter 1
    // BMA400_FIFO_8_BIT_EN     - Store data with only 8 bits instead of 12
    // BMA400_FIFO_X_EN         - Store x-axis data
    // BMA400_FIFO_Y_EN         - Store y-axis data
    // BMA400_FIFO_Z_EN         - Store z-axis data
    bma400_fifo_conf fifo_config =
    {
        .conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN,
        .conf_status = BMA400_ENABLE,
        .fifo_watermark = ACCEL_DATA_NUM_SAMPLES,
        .fifo_full_channel = BMA400_UNMAP_INT_PIN,
        .fifo_wm_channel = BMA400_INT_CHANNEL_1
    };
    _IMU.setFIFOConfig(&fifo_config);

    bma400_gen_int_conf int_config =
    {
        .gen_int_thres = 5, // 8mg resolution (eg. gen_int_thres=5 results in 40mg)
        .gen_int_dur = 5, // 10ms resolution (eg. gen_int_dur=5 results in 50ms)
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_2, // Which filter to use (must be 100Hz, datasheet recommends filter 2)
        .criterion_sel = BMA400_ACTIVITY_INT, // Trigger interrupts when active or inactive
        .evaluate_axes = BMA400_ANY_AXES_INT, // Logical combining of axes for interrupt condition (OR/AND)
        .ref_update = BMA400_UPDATE_EVERY_TIME, // Whether to automatically update reference values
        .hysteresis = BMA400_HYST_96_MG, // Hysteresis acceleration for noise rejection
        .int_thres_ref_x = 0, // Raw 12-bit acceleration value
        .int_thres_ref_y = 0, // Raw 12-bit acceleration value
        .int_thres_ref_z = 1024, // Raw 12-bit acceleration value (at 2g range, 1024 = 1g)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    _IMU.setGeneric1Interrupt(&int_config);

    // Here we configure the INT1 pin to push/pull mode, active high
    _IMU.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);

    // Enable FIFO watermark interrupt condition
    _IMU.enableInterrupt(BMA400_FIFO_WM_INT_EN, true);

    // Enable FIFO full interrupt condition
    _IMU.enableInterrupt(BMA400_FIFO_FULL_INT_EN, true);

    // Enable generic 1 interrupt condition
    _IMU.enableInterrupt(BMA400_GEN1_INT_EN, true);

    // Enable latching interrupts
    _IMU.enableInterrupt(BMA400_LATCH_INT_EN, true);

    // Remove initial control frame from FIFO data
    delay(100);
    _IMU.flushFIFO();

    // Clear interrupts
    uint16_t interruptStatus = 0;
    _IMU.getInterruptStatus(&interruptStatus);

    // Setup interrupt monitor
    nrf_gpio_cfg_sense_input(ACCEL_INT1_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  }
  return ( result == BMA400_OK );
}

bool AccelBma400::end()
{
  int8_t result = -128;
  {
    auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
    if ( ! p_sem ) { return false; }
    result = _IMU.setMode(BMA400_MODE_SLEEP);
  }
  zat.endQueue();
  return ( result == BMA400_OK );
}

bool AccelBma400::checkDevice()
{
  _console->print("Checking Accel: BMA400 ... ");

  auto p_sem = i2c.takeSemaphore(__PRETTY_FUNCTION__);
  if ( ! p_sem ) { return false; }
  int8_t result = _IMU.beginI2C(_addressI2c);

  if ( result == BMA400_OK ) {
    _console->println("SUCCESS");
    return true;
  } else {
    _console->println("FAILED");
    return false;
  }
}
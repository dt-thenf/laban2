/*
  GY85 Wearable Compass - ESP32 / Arduino
  Sensors: ADXL345 + ITG3205/ITG3200 + HMC5883L

  Goals:
  - Tilt-compensated heading for a wearable device.
  - Gyroscope-assisted gravity estimation for stable tilt compensation.
  - Hard-iron + basic diagonal soft-iron calibration on-device.
  - Optional full 3x3 soft-iron matrix from tools/calibrate_mag.py.
  - One-time physical arrow alignment saved in ESP32 NVS.
  - Magnetic declination saved in NVS.
  - Hold-last-heading near the mathematical singularity where the arrow points
    almost vertically and its horizontal projection becomes undefined.

  Serial monitor: 115200 baud, newline ending.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Hardware / timing
// -----------------------------------------------------------------------------

static constexpr uint8_t ADXL345_ADDR  = 0x53;
static constexpr uint8_t ITG3205_ADDR  = 0x68;
static constexpr uint8_t HMC5883L_ADDR = 0x1E;

static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;
static constexpr uint32_t I2C_FREQ_HZ = 400000;

static constexpr float LOOP_HZ = 100.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);
static constexpr uint32_t PRINT_INTERVAL_MS = 100;

// Heading is undefined when the physical arrow is nearly vertical.
// 0.17 ~= sin(9.8 deg), so we hold the last heading near this singularity.
static constexpr float MIN_FORWARD_HORIZONTAL = 0.17f;

// Magnetic disturbance gate after calibration.
static constexpr float MAG_NORM_MIN_RATIO = 0.55f;
static constexpr float MAG_NORM_MAX_RATIO = 1.55f;

// -----------------------------------------------------------------------------
// Small vector helpers
// -----------------------------------------------------------------------------

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

static inline Vec3 vAdd(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline Vec3 vSub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline Vec3 vScale(const Vec3& a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}

static inline float vDot(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline Vec3 vCross(const Vec3& a, const Vec3& b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

static inline float vNorm(const Vec3& a) {
  return sqrtf(vDot(a, a));
}

static inline Vec3 vNormalize(const Vec3& a) {
  float n = vNorm(a);
  if (n < 1.0e-9f) return {0.0f, 0.0f, 0.0f};
  return vScale(a, 1.0f / n);
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline float wrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

static inline float wrap180(float deg) {
  deg = wrap360(deg);
  if (deg > 180.0f) deg -= 360.0f;
  return deg;
}

// -----------------------------------------------------------------------------
// Axis mapping
// -----------------------------------------------------------------------------
// All 3 sensors must be expressed in ONE common body frame.
//
// The default maps are identity because many GY-85 boards/examples expose the
// three sensor axes with compatible orientation. Clone boards can differ.
// If heading changes strongly when tilting, verify these maps first.
//
// src[i] selects native X/Y/Z (0/1/2) for body X/Y/Z.
// sign[i] is +1 or -1.
//
// Example to map body X = native Y, body Y = -native X, body Z = native Z:
//   {{1,0,2}, {+1,-1,+1}}

struct AxisMap {
  uint8_t src[3];
  int8_t sign[3];
};

static constexpr AxisMap ACCEL_MAP = {{0,1,2}, {+1,+1,+1}};
static constexpr AxisMap GYRO_MAP  = {{0,1,2}, {+1,+1,+1}};
static constexpr AxisMap MAG_MAP   = {{0,1,2}, {+1,+1,+1}};

static Vec3 applyAxisMap(const Vec3& in, const AxisMap& m) {
  const float a[3] = {in.x, in.y, in.z};
  return {
    a[m.src[0]] * m.sign[0],
    a[m.src[1]] * m.sign[1],
    a[m.src[2]] * m.sign[2]
  };
}

// -----------------------------------------------------------------------------
// Persistent config in ESP32 NVS
// -----------------------------------------------------------------------------

static constexpr uint32_t CFG_MAGIC = 0x47593835UL; // "GY85"
static constexpr uint16_t CFG_VERSION = 3;

enum ConfigFlags : uint8_t {
  CFG_GYRO_CAL = 1 << 0,
  CFG_MAG_CAL  = 1 << 1,
  CFG_ARROW_SET = 1 << 2
};

struct PersistentConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  float gyroBiasDps[3];

  float magBias[3];
  float magMatrix[9];
  float magRefNorm;

  float declinationDeg;
  float arrowYawDeg;

  uint8_t flags;
  uint8_t reserved[3];

  uint32_t crc32;
};

Preferences prefs;
PersistentConfig cfg{};

static uint32_t crc32Bytes(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static void setDefaultConfig() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.size = sizeof(cfg);

  cfg.magMatrix[0] = 1.0f;
  cfg.magMatrix[4] = 1.0f;
  cfg.magMatrix[8] = 1.0f;
  cfg.magRefNorm = 0.0f;

  cfg.declinationDeg = 0.0f;
  cfg.arrowYawDeg = 0.0f;
  cfg.flags = 0;
}

static bool validateConfig(PersistentConfig candidate) {
  if (candidate.magic != CFG_MAGIC) return false;
  if (candidate.version != CFG_VERSION) return false;
  if (candidate.size != sizeof(PersistentConfig)) return false;

  uint32_t stored = candidate.crc32;
  candidate.crc32 = 0;
  uint32_t calc = crc32Bytes(reinterpret_cast<const uint8_t*>(&candidate),
                             sizeof(candidate));
  return stored == calc;
}

static bool loadConfig() {
  setDefaultConfig();
  prefs.begin("gy85cmp", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n != sizeof(PersistentConfig)) {
    prefs.end();
    return false;
  }

  PersistentConfig candidate{};
  prefs.getBytes("cfg", &candidate, sizeof(candidate));
  prefs.end();

  if (!validateConfig(candidate)) return false;
  cfg = candidate;
  return true;
}

static bool saveConfig() {
  cfg.magic = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.size = sizeof(PersistentConfig);
  cfg.crc32 = 0;
  cfg.crc32 = crc32Bytes(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg));

  prefs.begin("gy85cmp", false);
  size_t n = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  return n == sizeof(cfg);
}

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (got != len) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < len; ++i) dst[i] = Wire.read();
  return true;
}

// -----------------------------------------------------------------------------
// Sensor drivers
// -----------------------------------------------------------------------------

struct SensorStatus {
  bool accel = false;
  bool gyro = false;
  bool mag = false;
};

SensorStatus sensorStatus;

static bool initADXL345() {
  if (!i2cProbe(ADXL345_ADDR)) return false;

  // BW_RATE = 100 Hz.
  if (!writeReg(ADXL345_ADDR, 0x2C, 0x0A)) return false;

  // DATA_FORMAT: FULL_RES=1, +/-2g.
  if (!writeReg(ADXL345_ADDR, 0x31, 0x08)) return false;

  // POWER_CTL: measurement mode.
  if (!writeReg(ADXL345_ADDR, 0x2D, 0x08)) return false;
  delay(10);
  return true;
}

static bool initITG3205() {
  if (!i2cProbe(ITG3205_ADDR)) return false;

  // Reset, then wake.
  writeReg(ITG3205_ADDR, 0x3E, 0x80);
  delay(20);
  if (!writeReg(ITG3205_ADDR, 0x3E, 0x00)) return false;

  // Internal sample = 1 kHz with DLPF enabled. Divider 9 -> 100 Hz.
  if (!writeReg(ITG3205_ADDR, 0x15, 9)) return false;

  // FS_SEL=3 (+/-2000 dps), DLPF_CFG=3 (~42 Hz).
  if (!writeReg(ITG3205_ADDR, 0x16, 0x1B)) return false;

  delay(50);
  return true;
}

static bool initHMC5883L() {
  if (!i2cProbe(HMC5883L_ADDR)) return false;

  // Optional identity check. Genuine HMC5883L normally returns 'H','4','3'.
  uint8_t id[3] = {0};
  bool idOk = readRegs(HMC5883L_ADDR, 0x0A, id, 3);
  if (idOk && !(id[0] == 'H' && id[1] == '4' && id[2] == '3')) {
    Serial.printf("WARN,HMC_ID,%02X,%02X,%02X\n", id[0], id[1], id[2]);
  }

  // Config A: 8-sample averaging, 75 Hz, normal measurement.
  if (!writeReg(HMC5883L_ADDR, 0x00, 0x78)) return false;

  // Config B: gain = +/-1.3 gauss.
  if (!writeReg(HMC5883L_ADDR, 0x01, 0x20)) return false;

  // Continuous measurement.
  if (!writeReg(HMC5883L_ADDR, 0x02, 0x00)) return false;
  delay(10);
  return true;
}

static bool readAccel(Vec3& outG) {
  uint8_t b[6];
  if (!readRegs(ADXL345_ADDR, 0x32, b, sizeof(b))) return false;

  int16_t rx = (int16_t)((uint16_t)b[1] << 8 | b[0]);
  int16_t ry = (int16_t)((uint16_t)b[3] << 8 | b[2]);
  int16_t rz = (int16_t)((uint16_t)b[5] << 8 | b[4]);

  // Full-resolution mode is approximately 3.9 mg/LSB.
  Vec3 native = {
    rx * 0.00390625f,
    ry * 0.00390625f,
    rz * 0.00390625f
  };
  outG = applyAxisMap(native, ACCEL_MAP);
  return true;
}

static bool readGyroRawDps(Vec3& outDps) {
  uint8_t b[6];
  if (!readRegs(ITG3205_ADDR, 0x1D, b, sizeof(b))) return false;

  int16_t rx = (int16_t)((uint16_t)b[0] << 8 | b[1]);
  int16_t ry = (int16_t)((uint16_t)b[2] << 8 | b[3]);
  int16_t rz = (int16_t)((uint16_t)b[4] << 8 | b[5]);

  // ITG-3200/3205 sensitivity at FS_SEL=3: 14.375 LSB/(deg/s).
  Vec3 native = {
    rx / 14.375f,
    ry / 14.375f,
    rz / 14.375f
  };
  outDps = applyAxisMap(native, GYRO_MAP);
  return true;
}

static bool readMagRaw(Vec3& outRaw) {
  uint8_t b[6];
  if (!readRegs(HMC5883L_ADDR, 0x03, b, sizeof(b))) return false;

  // HMC5883L register order is X, Z, Y.
  int16_t rx = (int16_t)((uint16_t)b[0] << 8 | b[1]);
  int16_t rz = (int16_t)((uint16_t)b[2] << 8 | b[3]);
  int16_t ry = (int16_t)((uint16_t)b[4] << 8 | b[5]);

  // -4096 indicates overflow/saturation.
  if (rx == -4096 || ry == -4096 || rz == -4096) return false;

  Vec3 native = {(float)rx, (float)ry, (float)rz};
  outRaw = applyAxisMap(native, MAG_MAP);
  return true;
}

// -----------------------------------------------------------------------------
// Gyro-assisted gravity estimator
// -----------------------------------------------------------------------------
// We estimate the world "up" vector expressed in body coordinates.
// This avoids Euler-angle singularities for tilt compensation.
//
// Prediction: fixed world vector in a rotating body frame obeys
//   du/dt = -omega x u = u x omega
//
// Accelerometer correction is trusted most when |a| ~= 1g.

class GravityEstimator {
public:
  bool initialized = false;
  Vec3 upBody{0.0f, 0.0f, 1.0f};

  void resetFromAccel(const Vec3& accelG) {
    if (vNorm(accelG) < 0.2f) return;
    upBody = vNormalize(accelG);
    initialized = true;
  }

  void update(const Vec3& accelG, const Vec3& gyroDps, float dt) {
    if (!initialized) {
      resetFromAccel(accelG);
      if (!initialized) return;
    }

    const float DEG2RAD = 0.01745329251994329577f;
    Vec3 omega = vScale(gyroDps, DEG2RAD);

    // Gyro prediction.
    Vec3 du = vCross(upBody, omega);
    upBody = vNormalize(vAdd(upBody, vScale(du, dt)));

    // Accelerometer correction.
    float amag = vNorm(accelG);
    if (amag > 0.55f && amag < 1.45f) {
      Vec3 aHat = vNormalize(accelG);

      // Confidence falls to zero as dynamic acceleration departs from 1g.
      float dev = fabsf(amag - 1.0f);
      float confidence = 1.0f - clampf(dev / 0.25f, 0.0f, 1.0f);
      confidence *= confidence;

      // ~0.45 s correction time constant at full confidence.
      float alpha = (1.0f - expf(-dt / 0.45f)) * confidence;
      upBody = vNormalize(vAdd(vScale(upBody, 1.0f - alpha),
                               vScale(aHat, alpha)));
    }
  }
};

GravityEstimator gravityEstimator;

// -----------------------------------------------------------------------------
// Magnetometer calibration / heading
// -----------------------------------------------------------------------------

static Vec3 applyMagCalibration(const Vec3& raw) {
  Vec3 d = {
    raw.x - cfg.magBias[0],
    raw.y - cfg.magBias[1],
    raw.z - cfg.magBias[2]
  };

  const float* m = cfg.magMatrix;
  return {
    m[0]*d.x + m[1]*d.y + m[2]*d.z,
    m[3]*d.x + m[4]*d.y + m[5]*d.z,
    m[6]*d.x + m[7]*d.y + m[8]*d.z
  };
}

static Vec3 arrowVectorBody(float arrowYawDeg) {
  float r = arrowYawDeg * 0.01745329251994329577f;
  return {cosf(r), sinf(r), 0.0f};
}

enum class HeadingQuality {
  OK,
  NO_MAG_CAL,
  MAG_DISTURBANCE,
  VERTICAL_HOLD,
  SENSOR_ERROR
};

static const char* qualityText(HeadingQuality q) {
  switch (q) {
    case HeadingQuality::OK: return "OK";
    case HeadingQuality::NO_MAG_CAL: return "NO_MAG_CAL";
    case HeadingQuality::MAG_DISTURBANCE: return "MAG_DISTURBANCE";
    case HeadingQuality::VERTICAL_HOLD: return "VERTICAL_HOLD";
    default: return "SENSOR_ERROR";
  }
}

struct HeadingResult {
  bool valid = false;
  float headingDeg = 0.0f;
  float magNorm = 0.0f;
  float forwardHorizontal = 0.0f;
  HeadingQuality quality = HeadingQuality::SENSOR_ERROR;
};

static HeadingResult computeHeading(const Vec3& magRaw, float arrowYawDeg) {
  HeadingResult r;
  if (!gravityEstimator.initialized) return r;

  Vec3 up = vNormalize(gravityEstimator.upBody);
  Vec3 mag = applyMagCalibration(magRaw);
  r.magNorm = vNorm(mag);
  if (r.magNorm < 1.0e-6f) return r;

  // Reject magnetic anomalies after a reference norm exists.
  if ((cfg.flags & CFG_MAG_CAL) && cfg.magRefNorm > 1.0e-6f) {
    float ratio = r.magNorm / cfg.magRefNorm;
    if (ratio < MAG_NORM_MIN_RATIO || ratio > MAG_NORM_MAX_RATIO) {
      r.quality = HeadingQuality::MAG_DISTURBANCE;
      return r;
    }
  }

  // Horizontal magnetic north in body coordinates.
  Vec3 northH = vSub(mag, vScale(up, vDot(mag, up)));
  float nNorm = vNorm(northH);
  if (nNorm < 1.0e-6f) return r;
  northH = vScale(northH, 1.0f / nNorm);

  // Physical arrow projected onto the horizontal plane.
  Vec3 f = arrowVectorBody(arrowYawDeg);
  Vec3 forwardH = vSub(f, vScale(up, vDot(f, up)));
  r.forwardHorizontal = vNorm(forwardH);

  if (r.forwardHorizontal < MIN_FORWARD_HORIZONTAL) {
    r.quality = HeadingQuality::VERTICAL_HOLD;
    return r;
  }
  forwardH = vScale(forwardH, 1.0f / r.forwardHorizontal);

  // Signed clockwise compass angle from magnetic north to physical arrow.
  // For a right-handed body frame, +rotation about Up is CCW, while compass
  // heading grows clockwise. This expression gives compass-positive heading.
  float sinAngle = vDot(up, vCross(forwardH, northH));
  float cosAngle = vDot(northH, forwardH);
  float heading = atan2f(sinAngle, cosAngle) * 57.29577951308232f;

  // Convert magnetic north to true north if declination is configured.
  heading = wrap360(heading + cfg.declinationDeg);

  r.valid = true;
  r.headingDeg = heading;
  r.quality = (cfg.flags & CFG_MAG_CAL)
            ? HeadingQuality::OK
            : HeadingQuality::NO_MAG_CAL;
  return r;
}

// Circular heading filter.
static bool headingFilterInit = false;
static float headingFilteredDeg = 0.0f;

static void resetHeadingFilter(float headingDeg) {
  headingFilteredDeg = wrap360(headingDeg);
  headingFilterInit = true;
}

static float updateHeadingFilter(float rawDeg) {
  if (!headingFilterInit) {
    resetHeadingFilter(rawDeg);
    return headingFilteredDeg;
  }

  float err = wrap180(rawDeg - headingFilteredDeg);
  float dynamic = clampf(fabsf(err) / 45.0f, 0.0f, 1.0f);
  float alpha = 0.04f + 0.18f * dynamic;
  headingFilteredDeg = wrap360(headingFilteredDeg + alpha * err);
  return headingFilteredDeg;
}

// -----------------------------------------------------------------------------
// Calibration state
// -----------------------------------------------------------------------------

static bool magCalRunning = false;
static Vec3 magMin{0,0,0};
static Vec3 magMax{0,0,0};
static uint32_t magCalSamples = 0;

static bool magStream = false;
static bool rawStream = false;

static void startMagCalibration(const Vec3& first) {
  magCalRunning = true;
  magMin = first;
  magMax = first;
  magCalSamples = 0;
  Serial.println("MAG_CAL,STARTED,rotate_figure8_and_all_3_axes_30_to_60_seconds");
}

static void updateMagCalibration(const Vec3& m) {
  if (!magCalRunning) return;

  magMin.x = min(magMin.x, m.x);
  magMin.y = min(magMin.y, m.y);
  magMin.z = min(magMin.z, m.z);
  magMax.x = max(magMax.x, m.x);
  magMax.y = max(magMax.y, m.y);
  magMax.z = max(magMax.z, m.z);
  ++magCalSamples;
}

static bool finishMagCalibration() {
  if (!magCalRunning) return false;
  magCalRunning = false;

  Vec3 radius = vScale(vSub(magMax, magMin), 0.5f);
  Vec3 bias = vScale(vAdd(magMax, magMin), 0.5f);

  if (magCalSamples < 300 ||
      radius.x < 80.0f || radius.y < 80.0f || radius.z < 80.0f) {
    Serial.printf("MAG_CAL,FAILED,samples=%lu,rx=%.1f,ry=%.1f,rz=%.1f\n",
                  (unsigned long)magCalSamples, radius.x, radius.y, radius.z);
    return false;
  }

  float avgRadius = (radius.x + radius.y + radius.z) / 3.0f;

  cfg.magBias[0] = bias.x;
  cfg.magBias[1] = bias.y;
  cfg.magBias[2] = bias.z;

  memset(cfg.magMatrix, 0, sizeof(cfg.magMatrix));
  cfg.magMatrix[0] = avgRadius / radius.x;
  cfg.magMatrix[4] = avgRadius / radius.y;
  cfg.magMatrix[8] = avgRadius / radius.z;
  cfg.magRefNorm = avgRadius;
  cfg.flags |= CFG_MAG_CAL;

  bool ok = saveConfig();
  Serial.printf("MAG_CAL,%s,bias=%.3f,%.3f,%.3f,scale=%.6f,%.6f,%.6f,ref=%.3f\n",
                ok ? "SAVED" : "SAVE_FAILED",
                cfg.magBias[0], cfg.magBias[1], cfg.magBias[2],
                cfg.magMatrix[0], cfg.magMatrix[4], cfg.magMatrix[8],
                cfg.magRefNorm);
  return ok;
}

// -----------------------------------------------------------------------------
// Runtime sensor state
// -----------------------------------------------------------------------------

Vec3 accelG;
Vec3 gyroRawDps;
Vec3 gyroDps;
Vec3 magRaw;

bool accelOk = false;
bool gyroOk = false;
bool magOk = false;

HeadingResult lastHeadingResult;

static Vec3 currentGyroBias() {
  return {cfg.gyroBiasDps[0], cfg.gyroBiasDps[1], cfg.gyroBiasDps[2]};
}

static void applyGyroBias() {
  gyroDps = vSub(gyroRawDps, currentGyroBias());
}

static bool calibrateGyroBlocking(uint16_t samples = 600) {
  Serial.printf("GYRO_CAL,START,%u samples,keep_device_still\n", samples);
  delay(250);

  Vec3 sum{0,0,0};
  uint16_t good = 0;

  for (uint16_t i = 0; i < samples; ++i) {
    Vec3 g;
    if (readGyroRawDps(g)) {
      sum = vAdd(sum, g);
      ++good;
    }
    delay(3);
  }

  if (good < samples * 0.8f) {
    Serial.printf("GYRO_CAL,FAILED,good=%u\n", good);
    return false;
  }

  Vec3 b = vScale(sum, 1.0f / good);
  if (vNorm(b) > 80.0f) {
    Serial.printf("GYRO_CAL,FAILED,bias_too_large=%.2f,%.2f,%.2f\n",
                  b.x,b.y,b.z);
    return false;
  }

  cfg.gyroBiasDps[0] = b.x;
  cfg.gyroBiasDps[1] = b.y;
  cfg.gyroBiasDps[2] = b.z;
  cfg.flags |= CFG_GYRO_CAL;

  bool ok = saveConfig();
  Serial.printf("GYRO_CAL,%s,bias_dps=%.5f,%.5f,%.5f\n",
                ok ? "SAVED" : "SAVE_FAILED", b.x,b.y,b.z);
  return ok;
}

// Slow runtime gyro bias adaptation when the unit is very still.
static float stationarySeconds = 0.0f;

static void updateRuntimeGyroBias(float dt) {
  float aNorm = vNorm(accelG);
  float correctedGNorm = vNorm(gyroDps);

  bool stationary = fabsf(aNorm - 1.0f) < 0.035f && correctedGNorm < 1.5f;
  if (!stationary) {
    stationarySeconds = 0.0f;
    return;
  }

  stationarySeconds += dt;
  if (stationarySeconds < 1.5f) return;

  // Runtime-only adaptation to temperature drift. We deliberately do not save
  // continuously to NVS to avoid flash wear.
  float alpha = clampf(dt / 25.0f, 0.0f, 0.01f);
  cfg.gyroBiasDps[0] = (1.0f-alpha)*cfg.gyroBiasDps[0] + alpha*gyroRawDps.x;
  cfg.gyroBiasDps[1] = (1.0f-alpha)*cfg.gyroBiasDps[1] + alpha*gyroRawDps.y;
  cfg.gyroBiasDps[2] = (1.0f-alpha)*cfg.gyroBiasDps[2] + alpha*gyroRawDps.z;
}

// -----------------------------------------------------------------------------
// Serial command helpers
// -----------------------------------------------------------------------------

static int parseFloats(const String& s, float* values, int maxValues) {
  char buf[320];
  s.toCharArray(buf, sizeof(buf));

  int count = 0;
  char* token = strtok(buf, " ,\t");
  while (token && count < maxValues) {
    char* end = nullptr;
    float v = strtof(token, &end);
    if (end != token) values[count++] = v;
    token = strtok(nullptr, " ,\t");
  }
  return count;
}

static void printMatrix() {
  Serial.printf("MAG_BIAS,%.8f,%.8f,%.8f\n",
                cfg.magBias[0], cfg.magBias[1], cfg.magBias[2]);
  Serial.printf("MAG_MAT,%.9f,%.9f,%.9f\n",
                cfg.magMatrix[0], cfg.magMatrix[1], cfg.magMatrix[2]);
  Serial.printf("MAG_MAT,%.9f,%.9f,%.9f\n",
                cfg.magMatrix[3], cfg.magMatrix[4], cfg.magMatrix[5]);
  Serial.printf("MAG_MAT,%.9f,%.9f,%.9f\n",
                cfg.magMatrix[6], cfg.magMatrix[7], cfg.magMatrix[8]);
  Serial.printf("MAG_REF,%.8f\n", cfg.magRefNorm);
}

static void printStatus() {
  Serial.printf("SENSORS,ADXL345=%d,ITG3205=%d,HMC5883L=%d\n",
                sensorStatus.accel, sensorStatus.gyro, sensorStatus.mag);
  Serial.printf("CFG,gyro=%d,mag=%d,arrow=%d,decl=%.3f,arrowYaw=%.3f\n",
                !!(cfg.flags & CFG_GYRO_CAL),
                !!(cfg.flags & CFG_MAG_CAL),
                !!(cfg.flags & CFG_ARROW_SET),
                cfg.declinationDeg, cfg.arrowYawDeg);
  Serial.printf("GYRO_BIAS_DPS,%.6f,%.6f,%.6f\n",
                cfg.gyroBiasDps[0], cfg.gyroBiasDps[1], cfg.gyroBiasDps[2]);
  printMatrix();
  Serial.printf("UP,%.5f,%.5f,%.5f\n",
                gravityEstimator.upBody.x,
                gravityEstimator.upBody.y,
                gravityEstimator.upBody.z);

  if (headingFilterInit) {
    Serial.printf("HEADING_FILTERED,%.2f\n", headingFilteredDeg);
  }
}

static void printHelp() {
  Serial.println("COMMANDS:");
  Serial.println("  HELP");
  Serial.println("  STATUS");
  Serial.println("  SCAN");
  Serial.println("  GYRO_CAL                 # keep device still ~2 s; saves bias");
  Serial.println("  MAG_CAL_START            # rotate figure-8 + all faces");
  Serial.println("  MAG_CAL_STOP             # basic hard-iron + diagonal soft-iron");
  Serial.println("  MAG_STREAM 1|0           # print MAGCSV,x,y,z for Python tool");
  Serial.println("  RAW 1|0                  # stream accel/gyro/mag raw values");
  Serial.println("  MAG_MATRIX bX bY bZ m00 m01 m02 m10 m11 m12 m20 m21 m22 ref");
  Serial.println("  MAG_RESET");
  Serial.println("  DECL <degrees>            # east positive, west negative");
  Serial.println("  ARROW_SET <bearing_deg>   # device nearly level; saves one-time arrow");
  Serial.println("  ARROW_RESET");
  Serial.println("  RESET_ALL");
}

static void scanI2C() {
  Serial.println("I2C_SCAN,BEGIN");
  for (uint8_t a = 1; a < 127; ++a) {
    if (i2cProbe(a)) Serial.printf("I2C,0x%02X\n", a);
  }
  Serial.println("I2C_SCAN,END");
}

static bool setArrowFromKnownBearing(float knownBearingDeg) {
  if (!magOk || !gravityEstimator.initialized) {
    Serial.println("ARROW_SET,FAILED,no_valid_heading");
    return false;
  }

  // The arrow lies in the PCB XY plane, so one-time yaw offset calibration is
  // best done with the PCB approximately level.
  Vec3 up = vNormalize(gravityEstimator.upBody);
  float levelness = fabsf(up.z);
  if (levelness < 0.94f) {
    Serial.printf("ARROW_SET,FAILED,device_not_level,abs_up_z=%.3f\n", levelness);
    return false;
  }

  HeadingResult base = computeHeading(magRaw, 0.0f);
  if (!base.valid) {
    Serial.printf("ARROW_SET,FAILED,quality=%s\n", qualityText(base.quality));
    return false;
  }

  cfg.arrowYawDeg = wrap180(base.headingDeg - knownBearingDeg);
  cfg.flags |= CFG_ARROW_SET;

  bool ok = saveConfig();
  if (ok) {
    resetHeadingFilter(wrap360(knownBearingDeg));
    Serial.printf("ARROW_SET,SAVED,known=%.3f,base=%.3f,arrowYaw=%.3f\n",
                  wrap360(knownBearingDeg), base.headingDeg, cfg.arrowYawDeg);
  } else {
    Serial.println("ARROW_SET,SAVE_FAILED");
  }
  return ok;
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "HELP") {
    printHelp();
    return;
  }

  if (upper == "STATUS") {
    printStatus();
    return;
  }

  if (upper == "SCAN") {
    scanI2C();
    return;
  }

  if (upper == "GYRO_CAL") {
    calibrateGyroBlocking();
    return;
  }

  if (upper == "MAG_CAL_START") {
    if (!magOk) {
      Serial.println("MAG_CAL,FAILED,no_magnetometer");
      return;
    }
    startMagCalibration(magRaw);
    return;
  }

  if (upper == "MAG_CAL_STOP") {
    finishMagCalibration();
    return;
  }

  if (upper.startsWith("MAG_STREAM")) {
    float v[1];
    int n = parseFloats(line.substring(String("MAG_STREAM").length()), v, 1);
    if (n == 1) magStream = (v[0] != 0.0f);
    Serial.printf("MAG_STREAM,%d\n", magStream);
    return;
  }

  if (upper.startsWith("RAW")) {
    float v[1];
    int n = parseFloats(line.substring(3), v, 1);
    if (n == 1) rawStream = (v[0] != 0.0f);
    Serial.printf("RAW,%d\n", rawStream);
    return;
  }

  if (upper.startsWith("MAG_MATRIX")) {
    float v[13];
    int n = parseFloats(line.substring(String("MAG_MATRIX").length()), v, 13);
    if (n != 13) {
      Serial.printf("MAG_MATRIX,FAILED,need_13_values,got=%d\n", n);
      return;
    }

    cfg.magBias[0] = v[0];
    cfg.magBias[1] = v[1];
    cfg.magBias[2] = v[2];
    for (int i = 0; i < 9; ++i) cfg.magMatrix[i] = v[3+i];
    cfg.magRefNorm = fabsf(v[12]);
    if (cfg.magRefNorm < 1.0e-6f) cfg.magRefNorm = 1.0f;
    cfg.flags |= CFG_MAG_CAL;

    bool ok = saveConfig();
    Serial.printf("MAG_MATRIX,%s\n", ok ? "SAVED" : "SAVE_FAILED");
    printMatrix();
    return;
  }

  if (upper == "MAG_RESET") {
    cfg.magBias[0] = cfg.magBias[1] = cfg.magBias[2] = 0.0f;
    memset(cfg.magMatrix, 0, sizeof(cfg.magMatrix));
    cfg.magMatrix[0] = cfg.magMatrix[4] = cfg.magMatrix[8] = 1.0f;
    cfg.magRefNorm = 0.0f;
    cfg.flags &= ~CFG_MAG_CAL;
    saveConfig();
    Serial.println("MAG_RESET,DONE");
    return;
  }

  if (upper.startsWith("DECL")) {
    float v[1];
    int n = parseFloats(line.substring(4), v, 1);
    if (n != 1 || v[0] < -180.0f || v[0] > 180.0f) {
      Serial.println("DECL,FAILED,use_-180_to_180_degrees");
      return;
    }
    cfg.declinationDeg = v[0];
    bool ok = saveConfig();
    Serial.printf("DECL,%s,%.4f\n", ok ? "SAVED" : "SAVE_FAILED", cfg.declinationDeg);
    return;
  }

  if (upper.startsWith("ARROW_SET")) {
    float v[1];
    int n = parseFloats(line.substring(String("ARROW_SET").length()), v, 1);
    if (n != 1) {
      Serial.println("ARROW_SET,FAILED,example=ARROW_SET 0");
      return;
    }
    setArrowFromKnownBearing(wrap360(v[0]));
    return;
  }

  if (upper == "ARROW_RESET") {
    cfg.arrowYawDeg = 0.0f;
    cfg.flags &= ~CFG_ARROW_SET;
    saveConfig();
    Serial.println("ARROW_RESET,DONE");
    return;
  }

  if (upper == "RESET_ALL") {
    setDefaultConfig();
    saveConfig();
    headingFilterInit = false;
    Serial.println("RESET_ALL,DONE,reboot_recommended");
    return;
  }

  Serial.printf("UNKNOWN_COMMAND,%s\n", line.c_str());
}

// -----------------------------------------------------------------------------
// Human-readable direction
// -----------------------------------------------------------------------------

static const char* cardinal8(float deg) {
  static const char* names[8] = {"N","NE","E","SE","S","SW","W","NW"};
  int idx = (int)floorf((wrap360(deg) + 22.5f) / 45.0f) & 7;
  return names[idx];
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("GY85_WEARABLE_COMPASS,BOOT");

  bool loaded = loadConfig();
  Serial.printf("CFG,%s\n", loaded ? "LOADED" : "DEFAULTS");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(20);

  sensorStatus.accel = initADXL345();
  sensorStatus.gyro  = initITG3205();
  sensorStatus.mag   = initHMC5883L();

  Serial.printf("SENSORS,ADXL345=%d,ITG3205=%d,HMC5883L=%d\n",
                sensorStatus.accel, sensorStatus.gyro, sensorStatus.mag);

  if (!sensorStatus.accel || !sensorStatus.gyro || !sensorStatus.mag) {
    Serial.println("ERROR,missing_sensor; run SCAN and check wiring");
  }

  // Initialize gravity from accelerometer.
  for (int i = 0; i < 30; ++i) {
    Vec3 a;
    if (readAccel(a)) {
      accelG = a;
      gravityEstimator.resetFromAccel(a);
    }
    delay(10);
  }

  // Only force a blocking gyro calibration if none is stored yet.
  if (sensorStatus.gyro && !(cfg.flags & CFG_GYRO_CAL)) {
    calibrateGyroBlocking();
  }

  printHelp();
  printStatus();
}

void loop() {
  static uint32_t nextLoopUs = micros();
  static uint32_t lastMicros = micros();
  static uint32_t lastPrintMs = 0;

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLoopUs) < 0) {
    while (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      handleCommand(line);
    }
    delay(1);
    return;
  }
  nextLoopUs += LOOP_US;

  float dt = (nowUs - lastMicros) * 1.0e-6f;
  lastMicros = nowUs;
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / LOOP_HZ;

  accelOk = sensorStatus.accel && readAccel(accelG);
  gyroOk  = sensorStatus.gyro  && readGyroRawDps(gyroRawDps);
  magOk   = sensorStatus.mag   && readMagRaw(magRaw);

  if (gyroOk) applyGyroBias();

  if (accelOk && gyroOk) {
    gravityEstimator.update(accelG, gyroDps, dt);
    updateRuntimeGyroBias(dt);
    applyGyroBias();
  } else if (accelOk && !gravityEstimator.initialized) {
    gravityEstimator.resetFromAccel(accelG);
  }

  if (magOk) {
    updateMagCalibration(magRaw);

    if (magStream) {
      Serial.printf("MAGCSV,%.3f,%.3f,%.3f\n", magRaw.x, magRaw.y, magRaw.z);
    }

    lastHeadingResult = computeHeading(magRaw, cfg.arrowYawDeg);
    if (lastHeadingResult.valid) {
      updateHeadingFilter(lastHeadingResult.headingDeg);
    }
    // If invalid because vertical or magnetic disturbance, keep last filtered
    // heading instead of producing an arbitrary jump.
  }

  if (rawStream) {
    Serial.printf("RAW,a=%.4f,%.4f,%.4f,g=%.4f,%.4f,%.4f,m=%.1f,%.1f,%.1f\n",
                  accelG.x, accelG.y, accelG.z,
                  gyroDps.x, gyroDps.y, gyroDps.z,
                  magRaw.x, magRaw.y, magRaw.z);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;

    float outHeading = headingFilterInit ? headingFilteredDeg : 0.0f;
    const char* arrowState = (cfg.flags & CFG_ARROW_SET) ? "SET" : "UNSET";

    Serial.printf(
      "COMPASS,heading=%.2f,raw=%.2f,dir=%s,quality=%s,arrow=%s,"
      "magNorm=%.3f,fwdHoriz=%.3f,up=%.3f,%.3f,%.3f\n",
      outHeading,
      lastHeadingResult.headingDeg,
      cardinal8(outHeading),
      qualityText(lastHeadingResult.quality),
      arrowState,
      lastHeadingResult.magNorm,
      lastHeadingResult.forwardHorizontal,
      gravityEstimator.upBody.x,
      gravityEstimator.upBody.y,
      gravityEstimator.upBody.z
    );
  }

  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}

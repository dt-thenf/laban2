#pragma once

#include <Arduino.h>

// Types used by the .ino file are kept in a real header so Arduino's automatic
// prototype generator sees them before generating prototypes.

struct Vec3 {
  float x;
  float y;
  float z;

  Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct AxisMap {
  uint8_t src[3];
  int8_t sign[3];

  constexpr AxisMap(uint8_t s0, uint8_t s1, uint8_t s2,
                    int8_t sg0, int8_t sg1, int8_t sg2)
      : src{s0, s1, s2}, sign{sg0, sg1, sg2} {}
};

enum ConfigFlags : uint8_t {
  CFG_GYRO_CAL = 1 << 0,
  CFG_MAG_CAL = 1 << 1,
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

enum class HeadingQuality : uint8_t {
  OK,
  NO_MAG_CAL,
  MAG_DISTURBANCE,
  VERTICAL_HOLD,
  SENSOR_ERROR
};

struct HeadingResult {
  bool valid;
  float headingDeg;
  float magNorm;
  float forwardHorizontal;
  HeadingQuality quality;

  HeadingResult()
      : valid(false),
        headingDeg(0.0f),
        magNorm(0.0f),
        forwardHorizontal(0.0f),
        quality(HeadingQuality::SENSOR_ERROR) {}
};

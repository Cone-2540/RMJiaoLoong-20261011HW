#pragma once

#include <stdint.h>
#include "motor_config.hpp"

class Motor {
  public:
    Motor() : encoder_(0), total_counts_(0), speed_rpm_(0), current_raw_(0),
              temperature_(0), received_(false) {}

    bool receive(const uint8_t data[8], bool restart) {
        uint16_t encoder = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        if (encoder >= 8192) {
            return false;
        }
        if (received_ && !restart) {
            int32_t delta = static_cast<int32_t>(encoder) - encoder_;
            if (delta > 4096) {
                delta -= 8192;
            } else if (delta < -4096) {
                delta += 8192;
            }
            total_counts_ += delta;
        } else {
            total_counts_ = 0;
        }
        encoder_ = encoder;
        speed_rpm_ = decodeSigned(data[2], data[3]);
        current_raw_ = decodeSigned(data[4], data[5]);
        temperature_ = MotorConfig::has_temperature ? data[6] : 0;
        received_ = true;
        return true;
    }

    float angle() const {
        return MotorConfig::direction * static_cast<float>(total_counts_) *
               360.0f / 8192.0f / MotorConfig::gear_ratio;
    }

    float speed() const {
        return MotorConfig::direction * speed_rpm_ * 6.0f / MotorConfig::gear_ratio;
    }

    int16_t currentRaw() const { return current_raw_; }
    uint16_t encoder() const { return encoder_; }
    uint8_t temperature() const { return temperature_; }

  private:
    static int16_t decodeSigned(uint8_t high, uint8_t low) {
        uint16_t bits = (static_cast<uint16_t>(high) << 8) | low;
        int32_t value = bits >= 32768 ? static_cast<int32_t>(bits) - 65536 : bits;
        return static_cast<int16_t>(value);
    }

    uint16_t encoder_;
    int64_t total_counts_;
    int16_t speed_rpm_;
    int16_t current_raw_;
    uint8_t temperature_;
    bool received_;
};

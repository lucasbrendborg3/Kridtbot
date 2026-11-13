/**
 * \file error_code.hpp
 * \mainpage
 *    Contains the error codes of the actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE
#define MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum ErrorCode
   * \brief
   *    Strongly typed enum for all known actuator error codes
  */
  enum class ErrorCode: std::uint16_t {
    NO_ERROR = 0x0000,
    MOTOR_STALL = 0x0002,
    LOW_VOLTAGE = 0x0004,
    MOTOR_STALL_AND_LOW_VOLTAGE = 0x0006, // This is a combination of the two codes above
    OVERVOLTAGE = 0x0008,
    MOTOR_STALL_AND_OVERVOLTAGE = 0x000A, // This is a combination of the two codes above
    OVERCURRENT = 0x0010,
    MOTOR_STALL_AND_OVERCURRENT = 0x0012, // This is a combination of the two codes above
    OVERCURRENT_AND_OVERVOLTAGE = 0x0014, // This is a combination of the two codes above
    MOTOR_STALL_AND_OVERCURRENT_AND_OVERVOLTAGE = 0x0016, // This is a combination of the three codes above
    POWER_OVERRUN = 0x0040,
    SPEEDING = 0x0100,
    UNSPECIFIED_1 = 0x0200, // There is no error string specified for the following three codes
    UNSPECIFIED_2 = 0x0400,
    UNSPECIFIED_3 = 0x0800,
    OVERTEMPERATURE = 0x1000,
    ENCODER_CALIBRATION_ERROR = 0x2000
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE

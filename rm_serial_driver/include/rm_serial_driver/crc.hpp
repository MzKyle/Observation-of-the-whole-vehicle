// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>

namespace crc16
{
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
//uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
inline uint16_t CRC16_Byte(uint16_t crc, const uint8_t data);
uint16_t CRC16_Calc(const uint8_t *buf, std::size_t len, uint16_t crc);
bool CRC16_Verify(const uint8_t *buf, std::size_t len);
/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

}  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_

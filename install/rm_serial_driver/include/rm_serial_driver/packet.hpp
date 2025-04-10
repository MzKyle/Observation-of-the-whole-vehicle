// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-auto 1-aim 2-buff
  bool reset_tracker : 1;
  uint8_t is_play : 1;
  bool change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t game_time;  // (s) game time [0, 450]
  uint32_t timestamp;  // (ms) board time
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t state : 2;       // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;          // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 3-outpost 4-normal
  float x;                 // aim: robot-center || buff: rune-center
  float y;                 // aim: robot-center || buff: rune-center
  float z;                 // aim: robot-center || buff: rune-center
  float yaw;               // aim: robot-yaw || buff: rune-theta
  // spd = a*sin(w*t)+b || spd > 0 ==> clockwise
  float vx;  // aim: robot-vx || buff: rune spin speed param - a
  float vy;  // aim: robot-vy || buff: rune spin speed param - b
  float vz;  // aim: robot-vz || buff: rune spin speed param - w
  float v_yaw;
  float r1;
  float r2;
  float dz;
  float pitch;

  float wz=0;
  uint8_t notice=5;
  uint32_t cap_timestamp;  // (ms) frame capture time
  uint16_t t_offset;       // (ms) speed t offset
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_

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
  struct __attribute__((packed))  {
    float yaw;
    float pit;
    float rol;
  } eulr;
  uint8_t notice;
  float current_v; // m/s
  float yaw;
  float pitch;
  float roll;

  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-auto 1-aim 2-buff

  uint32_t  event_data; // 重要事件数据        
  uint16_t time;         
        
  uint32_t rfid;        //增益 
  uint16_t base_hp;      //基地血量
  uint16_t sentry_hp;     //哨兵血量       
  uint16_t  outpost_hp; //前哨战
  uint16_t  projectile_allowance_17mm;    //允许发弹量

  // float aim_x;
  // float aim_y;
  // float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  //uint8_t header = 0xA5;
   /* 控制命令 */
  float yaw; 
  float pitch;
  float roll=0;   
  uint8_t notice=5;  
  float vx; 
  float vy; 
  
  float wz=0; 
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

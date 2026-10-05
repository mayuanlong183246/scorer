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

  uint8_t robot_id; 
  uint8_t game_process;
  uint16_t remain_time;

  uint16_t current_hp;

  uint16_t projectile_allowance;

  uint16_t base_hp;
  uint8_t fortress_status;

  uint8_t friendly_supply_zone_non_exchange;
  
  uint8_t friendly_supply_zone_exchange;
  uint8_t friendly_fortress_gain_point;

  uint16_t checksum = 0;
} __attribute__((packed));

  //uint16_t shooter_heat;
  //uint8_t robot_hurt;
  //uint8_t center_gain_point;(RMUL)

  //uint8_t sentry_info;
  //uint16_t outpost_hp;
  //uint16_t enemy_outpost_hp;
  
  //float yaw_diff; 
  // uint16_t base_hp;
  // uint16_t enemy_base_hp;
  // uint16_t bullets_fired;
  // uint16_t our_outpost_hp;
  // uint16_t enemy_outpost_hp;
  // uint32_t 
  // rfid_status;


struct SendPacket
{
  uint8_t header = 0x5A;
  
  //速度消息
  float vx;
  float vy;
  float vz;
  uint8_t status;

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

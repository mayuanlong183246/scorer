// Copyright 2026 RMUC contributors
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{

TEST(SendPacket, HasFixedWireLayoutAndValidCrc)
{
  SendPacket packet;
  packet.vx = 0.25F;
  packet.vy = -0.50F;
  packet.vz = 0.75F;
  packet.status = 3U;

  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  const std::vector<uint8_t> bytes = toVector(packet);

  EXPECT_EQ(bytes.size(), 16U);
  EXPECT_EQ(bytes.front(), 0x5AU);
  EXPECT_TRUE(crc16::Verify_CRC16_Check_Sum(bytes.data(), bytes.size()));

  float decoded_vx = 0.0F;
  float decoded_vy = 0.0F;
  float decoded_vz = 0.0F;
  std::memcpy(&decoded_vx, bytes.data() + 1, sizeof(float));
  std::memcpy(&decoded_vy, bytes.data() + 5, sizeof(float));
  std::memcpy(&decoded_vz, bytes.data() + 9, sizeof(float));
  EXPECT_FLOAT_EQ(decoded_vx, packet.vx);
  EXPECT_FLOAT_EQ(decoded_vy, packet.vy);
  EXPECT_FLOAT_EQ(decoded_vz, packet.vz);
  EXPECT_EQ(bytes[13], packet.status);

  std::vector<uint8_t> damaged = bytes;
  damaged[5] ^= 0x01U;
  EXPECT_FALSE(crc16::Verify_CRC16_Check_Sum(damaged.data(), damaged.size()));
}

TEST(SendPacket, ZeroVelocityFrameIsValid)
{
  SendPacket packet;
  packet.vx = 0.0F;
  packet.vy = 0.0F;
  packet.vz = 0.0F;
  packet.status = 0U;

  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  const std::vector<uint8_t> bytes = toVector(packet);

  EXPECT_EQ(bytes.size(), 16U);
  EXPECT_TRUE(crc16::Verify_CRC16_Check_Sum(bytes.data(), bytes.size()));
  for (std::size_t index = 1; index < 13; ++index) {
    EXPECT_EQ(bytes[index], 0U);
  }
}

}  // namespace rm_serial_driver

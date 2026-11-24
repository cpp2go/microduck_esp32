// SPDX-License-Identifier: MIT
//
// Local IMU on ESP32-P4: TDK ICM-42688 over SPI.
//
// Fills the two IMU blocks of the Microduck observation contract
// (pollen-robotics/microduck duck-control/src/imu.rs ImuData):
//   gyro              — angular velocity in the trunk frame, rad/s;
//   projected gravity — gravity direction in the trunk frame, UNIT vector,
//                       upright = [0, 0, -1].
// Original robotd reads an imu_to_dxl@200 SFLP block on the Dynamixel bus;
// this firmware uses a board-local ICM-42688 + Mahony until that path exists.
#pragma once

namespace imu {

/// Bring up SPI, probe WHO_AM_I, soft-reset, enable accel+gyro LN.
/// Returns false if the chip does not answer (loop may keep synthetic IMU).
bool Init();

/// Read accel/gyro and step the attitude filter. Call once per 50 Hz tick.
void Poll();

/// Angular velocity in rad/s, trunk frame (last Poll).
const float *GyroRad();

/// Gravity direction in the trunk frame, unit vector, upright [0,0,-1].
const float *ProjGravity();

/// Raw accelerometer in g (last Poll) — diagnostics/shell only.
const float *AccelG();

} // namespace imu

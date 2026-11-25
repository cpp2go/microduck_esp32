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

/// Read accel/gyro and step the attitude filter. Call once per 50 Hz tick
/// (or let StartFastTask poll at 200 Hz and call Snapshot from the 50 Hz loop).
void Poll();

/// Start a 200 Hz IMU poll task (SPI stand-in for original imu_to_dxl@200).
/// Safe to call once after Init(); no-op if IMU offline.
bool StartFastTask();

void StopFastTask();
bool FastTaskRunning();

/// Copy latest gyro/gravity into caller buffers (thread-safe vs fast task).
void Snapshot(float gyro_out[3], float gravity_out[3]);

/// Angular velocity in rad/s, trunk frame (last Poll / fast tick).
const float *GyroRad();

/// Gravity direction in the trunk frame, unit vector, upright [0,0,-1].
const float *ProjGravity();

/// Raw accelerometer in g (last Poll) — diagnostics/shell only.
const float *AccelG();

/// Fast-task period stats (µs) since start; zeros if idle.
void FastStats(int64_t *ticks, int64_t *misses, int64_t *min_us, int64_t *max_us,
               int64_t *sum_us);

} // namespace imu

// Copyright 2026 ESP-Duck Authors
// SPDX-License-Identifier: MIT
//
// CoreS3 IMU: Bosch BMI270 (accel + gyro) on the internal I2C bus
// (SDA GPIO12, SCL GPIO11), plus a Mahony attitude filter.
//
// Produces the two IMU blocks of the Microduck observation contract
// (see upstream duck-control/src/imu.rs):
//   gyro            — angular velocity in the trunk frame, rad/s;
//   projected gravity — gravity direction in the trunk frame, UNIT vector,
//                       upright = [0, 0, -1].
// The trunk-frame axis mapping of the CoreS3's PCB is identity for now; a
// physical mount needs the axis/sign constants in imu.cpp re-checked.
#pragma once

namespace imu {

/// Bring up I2C, probe the BMI270 (0x68 / 0x69), load the init firmware and
/// enable accel+gyro. Returns false if the chip does not answer (the app may
/// keep running with synthetic IMU data — check the log).
bool Init();

/// Read the sensor and step the attitude filter. Call once per control tick
/// (50 Hz is plenty; the chip runs at 100/200 Hz internally).
void Poll();

/// Angular velocity in rad/s, trunk frame (last Poll).
const float *GyroRad();

/// Gravity direction in the trunk frame, unit vector, upright [0,0,-1]
/// (last Poll — the "projected gravity" block of the observation).
const float *ProjGravity();

/// Raw accelerometer reading in g (last Poll) — diagnostics/shell only.
const float *AccelG();

} // namespace imu

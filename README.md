# Kalman Filtering for Mobile Robot Localization (MATLAB)

Coursework from **CIE 482 – Path Planning & Navigation for Mobile Robots** (KFUPM, Spring 2026 / T252).

Two state-estimation exercises: a linear Kalman filter for 1D robot localization with an interactive GUI, and an Extended Kalman Filter fusing IMU data to estimate orientation and the magnetic field.

## 1. 1D robot localization – linear Kalman filter

`kalman_1d_localization/`

- `kalman_predict.m` – prediction step: `x = F x + B u`, `P = F P Fᵀ + Q`
- `kalman_update.m` – correction step: Kalman gain, state and covariance update
- `RobotLocalizationGUI.m` – interactive GUI: move the robot with the ←/→ arrow keys, set the initial estimate and variance with sliders, and watch the Gaussian belief and the true / estimated / measured position history update live. Noisy position measurements arrive every `z_Ts` steps.

```matlab
cd kalman_1d_localization
RobotLocalizationGUI
```

Effect of measurement noise:

![Measurement noise experiments](media/kf_measurement_noise.png)

Effect of initial estimate and initial variance on convergence:

![Initial variance experiments](media/kf_initial_variance.png)

## 2. EKF for IMU sensor fusion

`ekf_imu_fusion/ekf_loop.m` – the EKF loop that estimates roll, pitch, yaw and body-frame magnetic field (Bx, By, Bz) from gyroscope (input), accelerometer and magnetometer (measurements), including Jacobian linearisation, angle wrapping and the Normalized Innovation Squared (NIS) consistency check.

The loop plugs into the course-provided `system_model.m` and `ekf_imu_fusion_simulation.m`, which are not included here.

## Credits

Course instructor: Dr. Ramy Rashad. The GUI framework and function templates were provided as starter code; the Kalman filter and EKF implementations, GUI fixes and experiments are my own work.

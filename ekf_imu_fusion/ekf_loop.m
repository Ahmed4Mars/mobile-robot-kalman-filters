% ekf_loop.m
% EKF prediction/correction loop for IMU sensor fusion (CIE 482, Assignment 5).
%
% Estimates orientation (roll, pitch, yaw) and the body-frame magnetic field
% (Bx, By, Bz) -> 6-state vector x_hat.
%
% This loop is placed inside the course-provided ekf_imu_fusion_simulation.m,
% which supplies: f_fun, F_fun, G_fun, h_fun, H_fun (from system_model.m),
% gyro / accel / mag measurement data, Q_k, R_k, steps, and the initial
% x_hat / P_hat. Those provided files are not included in this repository.

for k = 1:steps-1
    % 1. Read gyroscope input
    u_k = gyro(:, k);
    % 2. Predict the next state using f_fun
    x_pred = f_fun(x_hat, u_k);
    % 3. Compute F_k and G_k
    F_k = F_fun(x_hat, u_k);
    G_k = G_fun(x_hat, u_k);
    % 4. Predict covariance
    P_pred = F_k * P_hat * F_k' + G_k * Q_k * G_k';
    % 5. Build measurement vector z_k
    z_k = [accel(:,k+1); mag(:,k+1)];
    % 6. Predict measurement using h_fun
    z_pred = h_fun(x_pred);
    % 7. Compute H_k
    H_k = H_fun(x_pred);
    % 8. Innovation
    y_k = z_k - z_pred;
    % 9. Innovation covariance
    S_k = H_k * P_pred * H_k' + R_k;
    % 10. Kalman gain
    K_k = P_pred * H_k' / S_k;
    % 11. Update the state estimate
    x_hat = x_pred + K_k * y_k;
    % 12. Wrap Euler angles between -pi and pi
    x_hat(1:3) = wrapToPi(x_hat(1:3));
    % 13. Update covariance
    P_hat = (eye(6) - K_k * H_k) * P_pred;
    % 14. Save estimate
    xhat_all(:,k+1) = x_hat;
    % 15. Compute NIS
    NIS(k+1) = y_k' * (S_k \ y_k);
end

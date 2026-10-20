function [x_new, P_new] = kalman_predict(x_prev, P_prev, u_k, F_k, B_k, Q_k)
% kalman_predict
%
% Performs the prediction step of the Kalman filter for a linear system.
%
% Inputs:
%   x_prev - Previous state estimate (nx1 vector)
%   P_prev - Previous estimate covariance (nxn matrix)
%   u_k    - Control input (mx1 vector)
%   F_k    - State transition matrix (nxn)
%   B_k    - Control input model matrix (nxm)
%   Q_k    - Process noise covariance matrix (nxn)
%
% Outputs:
%   x_new  - Predicted state estimate
%   P_new  - Predicted estimate covariance

    x_new = F_k * x_prev + B_k * u_k;
    P_new = F_k * P_prev * F_k' + Q_k;

end

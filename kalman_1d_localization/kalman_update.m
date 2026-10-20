function [x_updated, P_updated, K] = kalman_update(x_pred, P_pred, z, H, R)
%KALMAN_UPDATE Performs the correction (update) step of the linear Kalman filter.
%
% Inputs:
%   x_pred  - Predicted state estimate (nx1 vector)
%   P_pred  - Predicted state covariance (nxn matrix)
%   z       - Measurement vector (qx1 vector)
%   H       - Measurement matrix (qxn matrix)
%   R       - Measurement noise covariance (qxq matrix)
%
% Outputs:
%   x_updated - Updated state estimate (nx1 vector)
%   P_updated - Updated state covariance (nxn matrix)
%   K         - Kalman gain (nxq matrix)

    K = P_pred * H' / (H * P_pred * H' + R);
    x_updated = x_pred + K * (z - H * x_pred);
    P_updated = (eye(size(P_pred)) - K * H) * P_pred;

end

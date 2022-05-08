         %----------- This function executes path control ---------%

% Function Inputs:
  %--> "vc" (Current velocity), size : 1 x 3
  %--> "vs" (Desired velocity), size : 1 x 3
  %--> "pc" (Current position), size : 1 x 3
  %--> "ps" (Desired position), size : 1 x 3
  %--> "Rs" (Sphere radius), size : 1 x 1

% Function Outputs:
  %--> "G" (Gradient), size : 1 x 3

function [G] = Traj_con(vc, vs, pc, ps, dt, Rs, Mm)
% Last known error signal and last known control action
persistent e_old; persistent G_old; 
if(isempty(e_old) || isempty(G_old))
    e_old = zeros(1,3); G_old = zeros(1,3);
end

% Control gains
k = 50; kp = 200; ki = 0;

% Current error
e = k * (ps - pc) + (vs - vc);

% PI discrete control
G = e * (kp + (dt/2) * ki) + e_old * (-kp + (dt/2) * ki) + G_old;

% Update old variables
G_old = G; e_old = e;

% Add feedforward optimal control to account for drag
G = G + (0.5 * 1025 * 0.47 * pi * Rs^2 * abs(vc)) / (Mm * (4/3) * pi * Rs^3);
end
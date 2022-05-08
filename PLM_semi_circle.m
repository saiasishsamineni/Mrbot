%----------- This function plans the MRbot path on a semi-circle ---------%

% Function Inputs:
  %--> "Np" (Number of digitized samples / Number of voxels), size : 1 x 1
  %--> "Rsc" (Semi-circle radius), size : 1 x 1
  %--> "Rsphere" (Radius of MRbot), size : 1 x 1
  %--> "Rgc" (Radius of guidance corridor), size : 1 x Np

% Function Outputs:
  %--> "P" Path profile [x(j), y(j), z(j)], size : 3 x (Np + 1)
  %--> "V" Velocity profile [vx(j), vy(j), vz(j)], size : 3 x Np

function [P, V] = PLM_semi_circle(Np, Rsc, Rsphere, Rgc)
% Compute path by segmenting semi-circle
Tht = 0 : (pi / Np) : pi;
P = [Rsc * cos(Tht); Rsc * sin(Tht); zeros(1, Np + 1)];

% Compute velocity tajectory according to paper guidelines
Vo = 100000; ko = 0.1; Ro = 0.1;
V = zeros(3, Np + 1);
for i = 1 : Np
    gamma = atan2(P(2, i+1) - P(2, i), P(1, i+1) - P(1, i));
    V(:, i) = (Vo / (1 + (1 / (ko * Rsc)))) * ((Rgc(i) - Rsphere) / Ro) * ...
        [cos(gamma); sin(gamma); 0];
end
end
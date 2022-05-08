    %--- This class models the motion of MRbot inside blood vessel ---%
classdef MRbot
    properties(SetAccess = private)
        % Static properties
        Rsphere %--> MRbot radius (m)
        dens %--> Iron density (kg/m3)
        M %--> Sphere magnetization constant at saturation (A/m)

        % Dynamic properties
        pos %--> Particle/sphere position (m)
        vel %--> Particle/sphere velocity (m/s)
    end
    methods
        function obj = MRbot(r, m, raw, pos_init)
            % Initialize parameters
            obj.Rsphere = r; obj.M = m; obj.dens = raw;

            % Dynamic model initial conditions
            obj.pos = pos_init; obj.vel = [0, 0, 0]; 
        end  
        function p = getpos(obj)
            % Get MRbot position vector
            p = obj.pos; 
        end
        function v = getvel(obj)
            % Get MRbot velocity vector
            v = obj.vel; 
        end
        function obj = solve(obj, G, dT)
            % Calculate magnetic force
            Vol = (4/3) * pi * obj.Rsphere^3; %--> Volume (m3)
            F = obj.M * Vol * G - 0.5 * 1025 * 0.47 * pi * obj.Rsphere^2 * abs(obj.vel);
              %--> Total resultant force (Magnetic + Drag)

            % Compute acceleration
            acc = F / (obj.dens * Vol);

            % Update kinematic states using euler's formula
            obj.pos = obj.pos + dT * obj.vel;
            obj.vel = obj.vel + dT * acc;
        end
    end
end


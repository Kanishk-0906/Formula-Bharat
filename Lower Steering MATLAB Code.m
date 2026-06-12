clc; clear;

% Given Constants

wheelbase   = 1600;            % Wheelbase [mm]
track_width = 1300;            % Track width [mm]
turn_radius = 3000;            % Turning radius [mm]

w           = 14 * 25.4;       % Rack length [mm]
steer_ratio = 9;               % Steering ratio (1:9)
rack_travel = 4 * 25.4;        % Rack travel [mm]
L_tie       = 650;             % Tie rod length [mm]
L_arm       = 150;             % Steering arm length [mm]
r_pinion    = 25;              % Pinion radius [mm]
gear_ratio  = 9;               % Gear ratio (1:9)
m           = 2;               % Module [mm] 
T_steering = 20;               % Steering wheel torque [Nm]
Sy = 850;                      % Yield strength [MPa] = [N/mm^2]
FoS = 2;                       % Factor of Safety
S_allow = Sy / FoS;            % Allowable stress [N/mm^2]

% Inner wheel angle (larger - inner to turn)
delta_inner = atan( wheelbase / (turn_radius - (track_width / 2)) );  % [rad]

% Outer wheel angle (smaller - outer to turn)
delta_outer = atan( wheelbase / (turn_radius + (track_width / 2)) );  % [rad]

% Convert to degrees
delta_inner_deg = rad2deg(delta_inner);   % [deg]
delta_outer_deg = rad2deg(delta_outer);   % [deg]

% Pinion diameter for +- 180 deg of pinion rotation (360 deg)
d_pinion = (rack_travel * 1) / (1 * pi);   % [mm]


%% Rack Position Optimization (Distance from Front Axle)

x_vals = linspace(150, 170, 500);
error_vals = zeros(size(x_vals));

[~, idx] = min(error_vals);
optimal_x = x_vals(idx);

T_pinion = T_steering*gear_ratio;     % Torque at pinion after gear ratio[Nm]
T_pinion_mm = T_pinion*1000;          % [Nmm]
F_rack = T_pinion_mm/r_pinion;        % Force on rack from pinion[N] 

% Bending moment at center (simply supported, central load)
M = (F_rack * w) / 4;    % [Nmm]
V = F_rack / 2;          % Shear force[N]

% Critical diameter from bending stress
% sigma = 32*M / (pi * d^3) <= S_allow
d_bending = (32 * M / (pi * S_allow))^(1/3);   % [mm]

% Critical diameter from shear stress
% tau = 16*T / (pi * d^3) <= S_allow/2
% For transverse shear: tau = 4V/3A = 4V/(3*pi*(d/2)^2)
tau_allow = S_allow / 2;
d_shear = sqrt((16 * V) / (3 * pi * tau_allow)); % [mm]

% Von Mises combined - take the larger of the two
d_critical = max(d_bending, d_shear);

% Print
fprintf('Inner Wheel Angle: %.4f deg\n', delta_inner_deg);
fprintf('Outer Wheel Angle: %.4f deg\n', delta_outer_deg);
fprintf('Pinion Diameter: %.4f mm\n', d_pinion);
fprintf('Optimal Rack Distance from Front Axle: %.4f mm\n', optimal_x);
fprintf('Pinion Torque: %.2f Nm\n', T_pinion);
fprintf('Rack Force: %.2f N\n', F_rack);
fprintf('Bending Moment at Center: %.2f Nmm\n', M);
fprintf('Shear Force: %.2f N\n', V);
fprintf('Min Diameter (bending): %.4f mm\n', d_bending);
fprintf('Min Diameter (shear): %.4f mm\n', d_shear);
fprintf('Critical Minimum Diameter: %.4f mm\n', d_critical);
close all; clear; clc;
% Load vehicle data
g= 9.81;
data_vehicle;
data_tire;
data_motor;
data_brake;
data_aero;
% Constant speed: graphical solution
motor_working_points;
% Brake distributor
brake_distribution;
%Cruise Controller
control_struct;
% Simulation parameters — Straight-line Acceleration Maneuver
simu.t_end = 45;
simu.dt      = 1e-4;
simu.dt_save = 1e-2;
simu.v0      = 130/3.6;           % Initial vehicle speed [m/s] (~6 km/h creep start)
simu.mu      = 1.0;             % Road friction coefficient [-]
simu.IC      = [simu.v0,0,0,0,0,0,simu.v0./tir.Re*ones(1,4),zeros(1,8)];
% %% %%INPUTS
% input.t_delta = 0:0.1:simu.t_end;
% input.delta = interp1([ 0, 1, 1.2, simu.t_end], deg2rad([0, 0, 2.5,2.5]*180),input.t_delta);
% input.delta = smooth(input.delta,10);

% INPUTS — Steering Pad (δ ramps at 1°/s continuously)

input.t_delta = 0:simu.dt_save:simu.t_end;
input.vx = ones(size(input.t_delta)) * (130/3.6);
input.delta   = deg2rad(1 .* input.t_delta);          % 1°/s ramp, in radians
% % Sine dwell
% W = 2*pi*0.7;
% delta0 = deg2rad(90);
% t = 0:0.001:2*pi/W+0.5;
% for ii = 1:length(t)
%     if t(ii)>0 && t(ii)<=3/2*pi/W
%             delta(ii) = delta0*sin(W*t(ii));
%     elseif t(ii)>3/2*pi/W && t(ii)<=3/2*pi/W+0.5
%             delta(ii) = -delta0;
%     elseif t(ii)>3/2*pi/W+0.5 && t(ii)<=2*pi/W+0.5
%             delta(ii) = delta0*( sin(W*(t(ii)-(3/2*pi/W+0.5 -3/2*pi/W))) );
%     else
%             delta(ii) = 0;
%     end
% end
% input.t_delta= [ 0, 1+t, 3+t(end) ];
% input.delta= [ 0, delta, 0 ];

figure('Name','Steering Input','NumberTitle','off');
plot(input.t_delta, input.delta, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Steering angle \delta [rad]');
title('Steering Input (1^\circ/s ramp)');
xlim([min(input.t_delta), max(input.t_delta)]);
yticks = get(gca,'YTick');
set(gca,'FontSize',11);
box on;
%% RUN SIMULATION
out = sim('lateral_model.slx');

%--------------------------------------------------------------------------
% Extract outputs
%--------------------------------------------------------------------------
t     = out.t.Data;
Fx    = out.Fx.Data;       % [FxF, FxR] — longitudinal contact forces [N]
Fz    = out.Fz.Data;       % [FzF, FzR] — vertical contact forces [N]
ax    = out.ax.Data;       % longitudinal acceleration [m/s^2]
omega = out.omega.Data;    % [omegaF, omegaR] — wheel angular velocities [rad/s]
ay    = out.ay.Data;       % lateral acceleration [m/s^2]
v     = out.v.Data;        % vehicle speed magnitude [m/s]
beta  = out.beta.Data;     % sideslip angle [rad]
xG    = out.xG.Data;       % global X position [m]
yG    = out.yG.Data;       % global Y position [m]
psid  = out.psid.Data;     % yaw rate derivative? (as provided) [rad/s^2]
psidd = out.psidd.Data;    % yaw angular acceleration [rad/s^2]
psi   = out.psi.Data;      % yaw angle [rad]
kappa = out.kappa.Data;    % path curvature or steering curvature variable [-]
alpha = out.alpha.Data;    % slip angles (vector or per-axle) [rad]
omegad= out.omegad.Data;   % wheel angular accelerations [rad/s^2]
Fy    = out.Fy.Data;       % lateral contact forces [N]
T     = out.T.Data;        % applied torques (vector) [N*m]
delta = out.delta.Data;    % steering angle (vector or input) [rad]
% Plot trajectory
figure('Name','Vehicle Trajectory','NumberTitle','off');
plot(xG, yG, 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('X position [m]');
ylabel('Y position [m]');
title('Vehicle Global Trajectory');

% Lateral acceleration vs time
figure('Name','Lateral Acceleration','NumberTitle','off');
plot(t, ay, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Lateral acceleration a_y [m/s^2]');
title('Lateral Acceleration vs Time');

% Yaw rate derivative (psid) vs time
figure('Name','Yaw Rate Derivative','NumberTitle','off');
plot(t, psid, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate \psi_d [rad/s]');
title('Yaw Rate Derivative vs Time');

% Sideslip angle vs time
figure('Name','Sideslip Angle','NumberTitle','off');
plot(t, beta, 'k', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Sideslip angle \beta [rad]');
title('Sideslip Angle vs Time');

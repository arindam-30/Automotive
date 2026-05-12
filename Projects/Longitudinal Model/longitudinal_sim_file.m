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
simu.dt      = 1e-4;
simu.dt_save = 1e-2;
simu.v0      = 50/3.6;           % Initial vehicle speed [m/s] (~6 km/h creep start)
simu.w0      = simu.v0/tir.Re;  % Initial wheel angular velocity [rad/s]
simu.t_end   = 100;             % Simulation end time [s]
simu.mu      = 1.0;             % Road friction coefficient [-]
simu.alpha   = 0;               % Road grade angle [deg] (set to 0 if flat road intended)
simu.IC      = [simu.w0, simu.w0, simu.v0,0,0];
%% %%INPUTS
%LONGITUDINAL MODEL INPUTS
%input.t  = 0:0.1:simu.t_end;
%input.vx = linspace(6/3.6,vx_max,1001);

%CRUISE CONTROL INPUTS
input.t = [0 5 21.67 41.67 45.37 55.37];
input.vx = [50 50 110 110 70 70 ]/3.6;
%with wind velocity
% input.t = [0 5 10 20 50 55 65];
% input.u = [0 -20 0 -20 10 0 0];
% input.vx = ones(size(input.u))*(90/3.6);

%STRAIGHT LINE ACCELERATION INPUTS
input.Tm = interp1([0,                       1,                       11,            100], ...
                   [Fres(simu.v0,0)*tir.Re,  Fres(simu.v0,0)*tir.Re,  max(mot.Tm),  max(mot.Tm)], ...
                   input.t);
input.p = zeros(size(input.t));   % No braking pressure during acceleration

% %BRAKING INPUTS
% p_ref = 30; % [bar]
% input.Tm = zeros(size(input.t));
% input.p  = interp1([ 0, 1, 2, simu.t_end]   ,...
%                    [ 0, 0, 1,          1]*p_ref, input.t);

%% RUN SIMULATION
out = sim('longitudinal_model.slx');

%--------------------------------------------------------------------------
% Extract outputs
%--------------------------------------------------------------------------
t     = out.t;
Fx    = out.Fx;       % [FxF, FxR] — longitudinal contact forces [N]
Fz    = out.Fz;       % [FzF, FzR] — vertical contact forces [N]
ax    = out.ax;       % longitudinal acceleration [m/s^2]
vx    = out.vx;       % longitudinal vehicle velocity [m/s]
omega = out.omega;    % [omegaF, omegaR] — wheel angular velocities [rad/s]

%--------------------------------------------------------------------------
% Post-processing: wheel peripheral velocities
%--------------------------------------------------------------------------
vF = omega(:,1) * tir.Re;   % Front wheel peripheral velocity [m/s]
vR = omega(:,2) * tir.Re;   % Rear wheel peripheral velocity [m/s]

%--------------------------------------------------------------------------
% Plotting — 4 separate figures as required
%--------------------------------------------------------------------------

% --- Figure 1: Longitudinal velocity & wheel peripheral velocities ---
figure;
plot(t, vx,  'k-',  'LineWidth', 1.8, 'DisplayName', 'v_x'); hold on;
plot(t, vF,  'b--', 'LineWidth', 1.4, 'DisplayName', '\omega_F R_e');
plot(t, vR,  'r--', 'LineWidth', 1.4, 'DisplayName', '\omega_R R_e');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Longitudinal Velocity & Wheel Peripheral Velocities');
legend('Location', 'best'); grid on;
% --- Figure - Cruise Control Velocities
figure;
plot(t, vx,  'k-',  'LineWidth', 1.8, 'DisplayName', 'v_x'); hold on;
plot(input.t, input.vx,  'b--', 'LineWidth', 1.4, 'DisplayName', '\cc_vx');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Input Velocity & CC Velocity');
legend('Location', 'best'); grid on;

% --- Figure 2: Longitudinal acceleration ---
figure;
plot(t, ax, 'k-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('a_x [m/s^2]');
title('Longitudinal Acceleration');
grid on;

% --- Figure 3: Longitudinal contact forces ---
figure;
plot(t, Fx(:,1), 'b-',  'LineWidth', 1.8, 'DisplayName', 'F_{xF}'); hold on;
plot(t, Fx(:,2), 'r--', 'LineWidth', 1.8, 'DisplayName', 'F_{xR}');
xlabel('Time [s]'); ylabel('Force [N]');
title('Longitudinal Contact Forces');
legend('Location', 'best'); grid on;

% --- Figure 4: Vertical contact forces ---
figure;
plot(t, Fz(:,1), 'b-',  'LineWidth', 1.8, 'DisplayName', 'F_{zF}'); hold on;
plot(t, Fz(:,2), 'r--', 'LineWidth', 1.8, 'DisplayName', 'F_{zR}');
xlabel('Time [s]'); ylabel('Force [N]');
title('Vertical Contact Forces');
legend('Location', 'best'); grid on;
close all; clear; clc;
g = 9.80665;
g = 9.81;
data_vehicle;
data_tire;
data_motor;
data_brake;
data_aero;
% Constant speed: graphical solution
motor_working_points;
% Brake distributor
brake_distribution;
% Cruise Controller
control_struct;

susp.muF = 35;    % [kg]  front unsprung mass
susp.muR = 30;    % [kg]  rear unsprung mass
susp.kF  = 20000; % [N/m] front spring stiffness
susp.kR  = 18000; % [N/m] rear spring stiffness

% Vehicle data
l  = veh.l;   % [m] wheelbase
lF = veh.lF;  % [m] distance of vehicle CoM from front axle
lR = veh.lR;  % [m] distance of vehicle CoM from rear axle
m  = veh.m;   % [kg] vehicle mass
kt = tir.kz;  % [N/m] tyre radial stiffness
ct = tir.cz;  % [Ns/m] tyre radial damping

% Front axle (active in workspace for eigenvalue/FRF section)
mu = susp.muF;          % [kg] unsprung mass
ms = m*lR/l/2 - mu;    % [kg] sprung mass
ks = susp.kF;           % [N/m] spring stiffness

cs = 3000; % [Ns/m] damping coefficient

% System matrices (front axle)
M = [ms 0;  0  mu];
K = [ks -ks; -ks  ks+kt];
C = [cs -cs; -cs  cs+ct];

% State-space eigenvalue analysis
A = [-M\C  -M\K; eye(2)  zeros(2,2)];
[V, L] = eig(A);
w0  = imag(diag(L));
fn  = w0 / (2*pi);
fnI  = fn(1);   % body bounce frequency  [Hz]
fnII = fn(3);   % wheel hop frequency    [Hz]

VI  = V(3:4,1) ./ V(3,1);   % first  eigenmode
VII = V(3:4,3) ./ V(3,3);   % second eigenmode

% Linear frequency vector for FRF (original)
f = (0:0.01:25);
w = 2*pi*f;

ZsZr = zeros(1, length(w));
ZuZr = zeros(1, length(w));
for jj = 1:length(w)
    MM = (-M*w(jj)^2 + 1i*w(jj)*C + K) \ [0; 1i*w(jj)*ct + kt];
    ZsZr(jj) = MM(1);
    ZuZr(jj) = MM(2);
end

H_DI = -w.^2 .* ZsZr;                      % Discomfort Index FRF
H_RH = (1i*w*ct + kt) .* (ZuZr - 1);       % Road Holding FRF  (slide sign)
H_WS = ZsZr - ZuZr;                         % Working Space FRF

% ISO weighting function (linear f vector)
BB = [1 4 8 80; -6 0 0 -20]';
B(1,1) = 0;
for ii_f = 2:length(f)
    B(ii_f) = interp1(log10(BB(:,1)), BB(:,2), log10(f(ii_f)), 'linear', 'extrap');
end
B = 10.^(B/20);
H_DI_weighted = H_DI .* B;

%==========================================================================
%% SECTION 1 — FRF in LOG SCALE (slide 14/15 style)
%==========================================================================

% Log-spaced frequency vector: 0.01 to 100 Hz — captures BOTH modes
f_log = logspace(-2, 2, 3000);
w_log = 2*pi*f_log;

% Recompute on log-spaced vector
ZsZr_log = zeros(1, length(w_log));
ZuZr_log = zeros(1, length(w_log));
for jj = 1:length(w_log)
    MM = (-M*w_log(jj)^2 + 1i*w_log(jj)*C + K) \ [0; 1i*w_log(jj)*ct + kt];
    ZsZr_log(jj) = MM(1);
    ZuZr_log(jj) = MM(2);
end

% Transfer functions (slide 13 definitions)
H_DI_log = -w_log.^2 .* ZsZr_log;                   % [(m/s^2)/m]
H_RH_log = (1i*w_log*ct + kt) .* (ZuZr_log - 1);    % [N/m]
H_WS_log = ZsZr_log - ZuZr_log;                      % [m/m]

% ISO weighting on log-spaced vector
B_log = zeros(1, length(f_log));
for ii_f = 1:length(f_log)
    B_log(ii_f) = interp1(log10(BB(:,1)), BB(:,2), log10(f_log(ii_f)), 'linear', 'extrap');
end
B_log      = 10.^(B_log/20);
H_DI_log_w = H_DI_log .* B_log;

%--- Figure 1: Displacement TFs (linear + log) ----------------------------
figure('Name', 'Displacement TF - Sprung & Unsprung', 'NumberTitle', 'off');

subplot(1,2,1); hold on; grid on;
plot(f, abs(ZsZr), 'b-', 'LineWidth', 1.8, 'DisplayName', 'sprung mass z_s/z_r');
plot(f, abs(ZuZr), 'r-', 'LineWidth', 1.8, 'DisplayName', 'unsprung mass z_u/z_r');
xlabel('freq [Hz]'); ylabel('|Z/Z_r|');
title('Linear Scale');
legend('Location', 'best'); xlim([0 30]);

subplot(1,2,2); hold on; grid on;
plot(f_log, abs(ZsZr_log), 'b-', 'LineWidth', 1.8, 'DisplayName', 'sprung mass z_s/z_r');
plot(f_log, abs(ZuZr_log), 'r-', 'LineWidth', 1.8, 'DisplayName', 'unsprung mass z_u/z_r');
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|Z/Z_r|');
title('Log-Log Scale');
legend('Location', 'best'); xlim([1e-2 1e2]);

sgtitle('Quarter Car - Displacement Transfer Functions');

%--- Figure 2: DI / RH / WS (slide 15 style, log-log) --------------------
figure('Name', 'FRF - DI / RH / WS (Log Scale)', 'NumberTitle', 'off');

subplot(1,3,1); hold on; grid on;
plot(f_log, abs(H_DI_log), 'b-', 'LineWidth', 1.8);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{DI}| [(m/s^2)/m]');
title('Discomfort');
text(0.05, 0.88, '$\frac{DI(\omega)}{Z_r(\omega)}$', ...
    'Units', 'normalized', 'Interpreter', 'latex', 'FontSize', 11);
xlim([1e-2 1e2]);

subplot(1,3,2); hold on; grid on;
plot(f_log, abs(H_RH_log), 'b-', 'LineWidth', 1.8);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{RH}| [N/m]');
title('Road holding');
text(0.05, 0.88, '$\frac{RH(\omega)}{Z_r(\omega)}$', ...
    'Units', 'normalized', 'Interpreter', 'latex', 'FontSize', 11);
xlim([1e-2 1e2]);

subplot(1,3,3); hold on; grid on;
plot(f_log, abs(H_WS_log), 'b-', 'LineWidth', 1.8);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{WS}| [m/m]');
title('Working space');
text(0.05, 0.88, '$\frac{WS(\omega)}{Z_r(\omega)}$', ...
    'Units', 'normalized', 'Interpreter', 'latex', 'FontSize', 11);
xlim([1e-2 1e2]);

sgtitle('Passive Suspension - Transfer Functions w.r.t. Road Irregularity');

%--- Figure 3: ISO weighted DI --------------------------------------------
figure('Name', 'Discomfort Index - ISO Weighted', 'NumberTitle', 'off');
hold on; grid on;
plot(f_log, abs(H_DI_log),   'b-',  'LineWidth', 1.8, 'DisplayName', 'H_{DI} unweighted');
plot(f_log, abs(H_DI_log_w), 'r--', 'LineWidth', 1.8, 'DisplayName', 'H_{DI} ISO weighted');
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{DI}| [(m/s^2)/m]');
title('Discomfort Index FRF - Unweighted vs ISO Weighted');
legend('Location', 'best'); xlim([1e-2 1e2]);

%--- Console: natural frequencies and FRF peaks ---------------------------
fprintf('\n=== NATURAL FREQUENCIES (from eigenvalues) ===\n');
fprintf('Mode I  (body bounce): fnI  = %.3f Hz\n', fnI);
fprintf('Mode II (wheel hop):   fnII = %.3f Hz\n', fnII);

fprintf('\n=== FRF PEAK VALUES ===\n');
fprintf('%-32s  %12s  %10s\n', 'Metric', 'Peak value', 'at f [Hz]');
fprintf('%s\n', repmat('-',1,58));
metrics_list = {abs(H_DI_log), abs(H_RH_log), abs(H_WS_log), abs(H_DI_log_w)};
labels_list  = {'H_DI [(m/s^2)/m]','H_RH [N/m]','H_WS [m/m]','H_DI ISO weighted [(m/s^2)/m]'};
for im = 1:length(metrics_list)
    [pk, idx] = max(metrics_list{im});
    fprintf('%-32s  %12.4f  %10.3f\n', labels_list{im}, pk, f_log(idx));
end

%==========================================================================
%% SECTION 2 — DAMPING RATIO ANALYSIS & cs SWEEP
%==========================================================================

%--- Axle parameters -------------------------------------------------------
mu_F = susp.muF;  ms_F = m*lR/l/2 - mu_F;  ks_F = susp.kF;
mu_R = susp.muR;  ms_R = m*lF/l/2 - mu_R;  ks_R = susp.kR;

%--- Critical damping: cc = 2*m*w0 = 2*sqrt(m*k) -------------------------
% Front
wn1_F  = sqrt(ks_F / ms_F);          % body bounce natural freq [rad/s]
wn2_F  = sqrt((ks_F+kt) / mu_F);     % wheel hop natural freq   [rad/s]
cc1_F  = 2 * ms_F * wn1_F;           % [Ns/m]  Mode I critical damping
cc2_F  = 2 * mu_F * wn2_F;           % [Ns/m]  Mode II critical damping

% Rear
wn1_R  = sqrt(ks_R / ms_R);
wn2_R  = sqrt((ks_R+kt) / mu_R);
cc1_R  = 2 * ms_R * wn1_R;
cc2_R  = 2 * mu_R * wn2_R;

fprintf('\n=== CRITICAL DAMPING VALUES ===\n');
fprintf('%-6s  %10s  %10s  %10s  %10s\n', 'Axle', 'cc_I [Ns/m]', 'cc_II [Ns/m]', 'wn_I [Hz]', 'wn_II [Hz]');
fprintf('%s\n', repmat('-',1,55));
fprintf('%-6s  %10.1f  %10.1f  %10.3f  %10.3f\n', 'Front', cc1_F, cc2_F, wn1_F/(2*pi), wn2_F/(2*pi));
fprintf('%-6s  %10.1f  %10.1f  %10.3f  %10.3f\n', 'Rear',  cc1_R, cc2_R, wn1_R/(2*pi), wn2_R/(2*pi));

%--- cs sweep on log-spaced frequency vector ------------------------------
cs_vec = 100:100:8000;   % [Ns/m] fine sweep
n_cs   = length(cs_vec);

% Log-spaced frequency for sweep (same f_log, w_log as above)
% ISO weighting already built as B_log

peak_DI_F = zeros(n_cs,1);  peak_RH_F = zeros(n_cs,1);  peak_WS_F = zeros(n_cs,1);
peak_DI_R = zeros(n_cs,1);  peak_RH_R = zeros(n_cs,1);  peak_WS_R = zeros(n_cs,1);

for ic = 1:n_cs
    cs_i = cs_vec(ic);

    % --- Front ---
    M_F = [ms_F 0; 0 mu_F];
    K_F = [ks_F -ks_F; -ks_F ks_F+kt];
    C_F = [cs_i -cs_i; -cs_i cs_i+ct];
    Zs_F = zeros(1,length(w_log));  Zu_F = zeros(1,length(w_log));
    for jj = 1:length(w_log)
        MM = (-M_F*w_log(jj)^2 + 1i*w_log(jj)*C_F + K_F) \ [0; 1i*w_log(jj)*ct+kt];
        Zs_F(jj) = MM(1);  Zu_F(jj) = MM(2);
    end
    H_DI_Fi = (-w_log.^2 .* Zs_F) .* B_log;    % ISO weighted DI
    H_RH_Fi =  (1i*w_log*ct+kt) .* (Zu_F - 1);
    H_WS_Fi =  Zs_F - Zu_F;
    peak_DI_F(ic) = max(abs(H_DI_Fi));
    peak_RH_F(ic) = max(abs(H_RH_Fi));
    peak_WS_F(ic) = max(abs(H_WS_Fi));

    % --- Rear ---
    M_R = [ms_R 0; 0 mu_R];
    K_R = [ks_R -ks_R; -ks_R ks_R+kt];
    C_R = [cs_i -cs_i; -cs_i cs_i+ct];
    Zs_R = zeros(1,length(w_log));  Zu_R = zeros(1,length(w_log));
    for jj = 1:length(w_log)
        MM = (-M_R*w_log(jj)^2 + 1i*w_log(jj)*C_R + K_R) \ [0; 1i*w_log(jj)*ct+kt];
        Zs_R(jj) = MM(1);  Zu_R(jj) = MM(2);
    end
    H_DI_Ri = (-w_log.^2 .* Zs_R) .* B_log;
    H_RH_Ri =  (1i*w_log*ct+kt) .* (Zu_R - 1);
    H_WS_Ri =  Zs_R - Zu_R;
    peak_DI_R(ic) = max(abs(H_DI_Ri));
    peak_RH_R(ic) = max(abs(H_RH_Ri));
    peak_WS_R(ic) = max(abs(H_WS_Ri));
end

% Damping ratios for x-axis (Mode I — body bounce, dominant for comfort)
zeta_F = cs_vec / cc1_F;
zeta_R = cs_vec / cc1_R;

% Normalised score: lower peak = better
norm_fn   = @(x) (x - min(x)) / (max(x) - min(x));
score_F   = norm_fn(peak_DI_F) + norm_fn(peak_RH_F) + norm_fn(peak_WS_F);
score_R   = norm_fn(peak_DI_R) + norm_fn(peak_RH_R) + norm_fn(peak_WS_R);

[~, idx_best_F] = min(score_F);
[~, idx_best_R] = min(score_R);

cs_best_F    = cs_vec(idx_best_F);
cs_best_R    = cs_vec(idx_best_R);
zeta_best_F1 = cs_best_F / cc1_F;   % Mode I  damping ratio front
zeta_best_F2 = cs_best_F / cc2_F;   % Mode II damping ratio front
zeta_best_R1 = cs_best_R / cc1_R;
zeta_best_R2 = cs_best_R / cc2_R;

fprintf('\n=== BEST DAMPING COEFFICIENTS ===\n');
fprintf('%-6s  %10s  %10s  %10s\n', 'Axle', 'cs [Ns/m]', 'zeta_I', 'zeta_II');
fprintf('%s\n', repmat('-',1,42));
fprintf('%-6s  %10.0f  %10.3f  %10.3f\n', 'Front', cs_best_F, zeta_best_F1, zeta_best_F2);
fprintf('%-6s  %10.0f  %10.3f  %10.3f\n', 'Rear',  cs_best_R, zeta_best_R1, zeta_best_R2);

%--- Figure 4: Peak FRF vs zeta_I - FRONT --------------------------------
figure('Name', 'Front: Peak FRF vs Damping Ratio', 'NumberTitle', 'off');

subplot(3,1,1); hold on; grid on;
plot(zeta_F, peak_DI_F, 'b-', 'LineWidth', 1.8);
xline(zeta_best_F1, 'r--', sprintf('\\zeta_I = %.2f', zeta_best_F1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{DI}^{w}|  [(m/s^2)/m]');
title('Front - Discomfort Index (ISO weighted)');

subplot(3,1,2); hold on; grid on;
plot(zeta_F, peak_RH_F, 'b-', 'LineWidth', 1.8);
xline(zeta_best_F1, 'r--', sprintf('\\zeta_I = %.2f', zeta_best_F1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{RH}|  [N/m]');
title('Front - Road Holding');

subplot(3,1,3); hold on; grid on;
plot(zeta_F, peak_WS_F, 'b-', 'LineWidth', 1.8);
xline(zeta_best_F1, 'r--', sprintf('\\zeta_I = %.2f', zeta_best_F1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{WS}|  [m/m]');
title('Front - Working Space');

sgtitle(sprintf('Front Suspension: Peak FRF vs \\zeta_I  (best cs = %d Ns/m)', cs_best_F));

%--- Figure 5: Peak FRF vs zeta_I - REAR ---------------------------------
figure('Name', 'Rear: Peak FRF vs Damping Ratio', 'NumberTitle', 'off');

subplot(3,1,1); hold on; grid on;
plot(zeta_R, peak_DI_R, 'r-', 'LineWidth', 1.8);
xline(zeta_best_R1, 'b--', sprintf('\\zeta_I = %.2f', zeta_best_R1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{DI}^{w}|  [(m/s^2)/m]');
title('Rear - Discomfort Index (ISO weighted)');

subplot(3,1,2); hold on; grid on;
plot(zeta_R, peak_RH_R, 'r-', 'LineWidth', 1.8);
xline(zeta_best_R1, 'b--', sprintf('\\zeta_I = %.2f', zeta_best_R1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{RH}|  [N/m]');
title('Rear - Road Holding');

subplot(3,1,3); hold on; grid on;
plot(zeta_R, peak_WS_R, 'r-', 'LineWidth', 1.8);
xline(zeta_best_R1, 'b--', sprintf('\\zeta_I = %.2f', zeta_best_R1), 'LineWidth', 1.5);
xlabel('\zeta_I  (body bounce)'); ylabel('peak |H_{WS}|  [m/m]');
title('Rear - Working Space');

sgtitle(sprintf('Rear Suspension: Peak FRF vs \\zeta_I  (best cs = %d Ns/m)', cs_best_R));

%--- Figure 6: FRF curves at best cs - Front vs Rear (log scale) ---------
% Front at best cs
C_Fb = [cs_best_F -cs_best_F; -cs_best_F cs_best_F+ct];
Zs_Fb = zeros(1,length(w_log));  Zu_Fb = zeros(1,length(w_log));
for jj = 1:length(w_log)
    MM = (-(M_F)*w_log(jj)^2 + 1i*w_log(jj)*C_Fb + K_F) \ [0; 1i*w_log(jj)*ct+kt];
    Zs_Fb(jj) = MM(1);  Zu_Fb(jj) = MM(2);
end
H_DI_Fb_log =  (-w_log.^2 .* Zs_Fb) .* B_log;
H_RH_Fb_log =  (1i*w_log*ct+kt) .* (Zu_Fb - 1);
H_WS_Fb_log =  Zs_Fb - Zu_Fb;

% Rear at best cs
C_Rb = [cs_best_R -cs_best_R; -cs_best_R cs_best_R+ct];
Zs_Rb = zeros(1,length(w_log));  Zu_Rb = zeros(1,length(w_log));
for jj = 1:length(w_log)
    MM = (-M_R*w_log(jj)^2 + 1i*w_log(jj)*C_Rb + K_R) \ [0; 1i*w_log(jj)*ct+kt];
    Zs_Rb(jj) = MM(1);  Zu_Rb(jj) = MM(2);
end
H_DI_Rb_log =  (-w_log.^2 .* Zs_Rb) .* B_log;
H_RH_Rb_log =  (1i*w_log*ct+kt) .* (Zu_Rb - 1);
H_WS_Rb_log =  Zs_Rb - Zu_Rb;

figure('Name', 'FRF at Best cs - Front vs Rear (Log)', 'NumberTitle', 'off');

subplot(1,3,1); hold on; grid on;
plot(f_log, abs(H_DI_Fb_log), 'b-',  'LineWidth', 1.8, ...
    'DisplayName', sprintf('Front  cs=%d, \\zeta_I=%.2f', cs_best_F, zeta_best_F1));
plot(f_log, abs(H_DI_Rb_log), 'r--', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Rear   cs=%d, \\zeta_I=%.2f', cs_best_R, zeta_best_R1));
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{DI}^{w}| [(m/s^2)/m]');
title('Discomfort (ISO weighted)');
legend('Location', 'best'); xlim([1e-2 1e2]);

subplot(1,3,2); hold on; grid on;
plot(f_log, abs(H_RH_Fb_log), 'b-',  'LineWidth', 1.8, ...
    'DisplayName', sprintf('Front  cs=%d', cs_best_F));
plot(f_log, abs(H_RH_Rb_log), 'r--', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Rear   cs=%d', cs_best_R));
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{RH}| [N/m]');
title('Road Holding');
legend('Location', 'best'); xlim([1e-2 1e2]);

subplot(1,3,3); hold on; grid on;
plot(f_log, abs(H_WS_Fb_log), 'b-',  'LineWidth', 1.8, ...
    'DisplayName', sprintf('Front  cs=%d', cs_best_F));
plot(f_log, abs(H_WS_Rb_log), 'r--', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Rear   cs=%d', cs_best_R));
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{WS}| [m/m]');
title('Working Space');
legend('Location', 'best'); xlim([1e-2 1e2]);

sgtitle('FRF at Best Damping Coefficient: Front vs Rear');

%==========================================================================
%% SECTION 3 — INFLUENCE OF DAMPING RATIO h (slide 16 style)
%  One curve per h value, all overlaid on same axes
%  Figure A: Zs/Zr and Zu/Zr  (slide 16)
%  Figure B: DI / RH / WS     (slide 17)
%  Plotted for FRONT axle; rear follows same pattern
%==========================================================================

h_vec   = [0.25, 0.50, 0.75, 1.00, 1.25, 1.50];   % damping ratios to sweep
n_h     = length(h_vec);

% Colormap matching slide (blue -> cyan progression)
h_colors = [
    0.00  0.45  0.74;   % h=0.25  dark blue
    0.00  0.60  0.90;   % h=0.50  medium blue
    0.85  0.33  0.10;   % h=0.75  orange
    0.93  0.69  0.13;   % h=1.00  yellow
    0.47  0.67  0.19;   % h=1.25  green
    0.30  0.75  0.93;   % h=1.50  cyan
];

% Pre-allocate storage (each row = one h value, each col = one freq point)
Zs_h  = zeros(n_h, length(w_log));
Zu_h  = zeros(n_h, length(w_log));
HDI_h = zeros(n_h, length(w_log));
HRH_h = zeros(n_h, length(w_log));
HWS_h = zeros(n_h, length(w_log));

for ih = 1:n_h
    % cs from h: cc1_F = 2*ms_F*wn1_F (body bounce critical damping)
    cs_h  = h_vec(ih) * cc1_F;
    C_h   = [cs_h -cs_h; -cs_h cs_h+ct];

    for jj = 1:length(w_log)
        MM = (-M_F*w_log(jj)^2 + 1i*w_log(jj)*C_h + K_F) \ ...
             [0; 1i*w_log(jj)*ct + kt];
        Zs_h(ih,jj) = MM(1);
        Zu_h(ih,jj) = MM(2);
    end

    HDI_h(ih,:) = (-w_log.^2 .* Zs_h(ih,:)) .* B_log;   % ISO weighted
    HRH_h(ih,:) = (1i*w_log*ct + kt) .* (Zu_h(ih,:) - 1);
    HWS_h(ih,:) =  Zs_h(ih,:) - Zu_h(ih,:);
end

% Ideal sprung mass TF: 1 below fn1, rolls off as (fn1/f)^2 above
%   = low-pass filter at body bounce frequency
f_ideal   = f_log;
fn1_F     = wn1_F / (2*pi);
TF_ideal  = ones(1, length(f_ideal));
above     = f_ideal > fn1_F;
TF_ideal(above) = (fn1_F ./ f_ideal(above)).^2;

%--- Figure A: Zs/Zr and Zu/Zr overlaid for all h  (slide 16) -----------
figure('Name', 'Influence of Damping - Displacement TFs', 'NumberTitle', 'off');

% Sprung mass TF
subplot(1,2,1); hold on; grid on;
for ih = 1:n_h
    plot(f_log, abs(Zs_h(ih,:)), '-', ...
        'Color', h_colors(ih,:), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('h = %.2f', h_vec(ih)));
end
% Ideal dashed green line
plot(f_ideal, TF_ideal, 'g--', 'LineWidth', 1.4, 'DisplayName', 'Ideal');
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|Z_s / Z_r|');
title('sprung mass TF');
text(0.04, 0.90, '$\frac{Z_s}{Z_r}$', 'Units','normalized', ...
    'Interpreter','latex','FontSize',13);
legend('Location','southwest','FontSize',8);
xlim([1e-1 1e2]);

% Unsprung mass TF
subplot(1,2,2); hold on; grid on;
for ih = 1:n_h
    plot(f_log, abs(Zu_h(ih,:)), '-', ...
        'Color', h_colors(ih,:), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('h = %.2f', h_vec(ih)));
end
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|Z_u / Z_r|');
title('unsprung mass TF');
text(0.04, 0.90, '$\frac{Z_u}{Z_r}$', 'Units','normalized', ...
    'Interpreter','latex','FontSize',13);
legend('Location','southwest','FontSize',8);
xlim([1e-1 1e2]);

sgtitle('Passive Suspension - Influence of Suspension Damping (Front Axle)');

%--- Figure B: DI / RH / WS overlaid for all h  (slide 17) --------------
figure('Name', 'Influence of Damping - FRF Metrics', 'NumberTitle', 'off');

% Discomfort
subplot(1,3,1); hold on; grid on;
for ih = 1:n_h
    plot(f_log, abs(HDI_h(ih,:)), '-', ...
        'Color', h_colors(ih,:), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('h = %.2f', h_vec(ih)));
end
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{DI}^w|  [(m/s^2)/m]');
title('Discomfort');
text(0.04, 0.90, '$\frac{DI(\omega)}{Z_r(\omega)}$', 'Units','normalized', ...
    'Interpreter','latex','FontSize',11);
legend('Location','northwest','FontSize',8);
xlim([1e-1 1e2]);

% Road Holding
subplot(1,3,2); hold on; grid on;
for ih = 1:n_h
    plot(f_log, abs(HRH_h(ih,:)), '-', ...
        'Color', h_colors(ih,:), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('h = %.2f', h_vec(ih)));
end
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{RH}|  [N/m]');
title('Road holding');
text(0.04, 0.90, '$\frac{RH(\omega)}{Z_r(\omega)}$', 'Units','normalized', ...
    'Interpreter','latex','FontSize',11);
legend('Location','northwest','FontSize',8);
xlim([1e-1 1e2]);

% Working Space
subplot(1,3,3); hold on; grid on;
for ih = 1:n_h
    plot(f_log, abs(HWS_h(ih,:)), '-', ...
        'Color', h_colors(ih,:), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('h = %.2f', h_vec(ih)));
end
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('freq [Hz]'); ylabel('|H_{WS}|  [m/m]');
title('Working space');
text(0.04, 0.90, '$\frac{WS(\omega)}{Z_r(\omega)}$', 'Units','normalized', ...
    'Interpreter','latex','FontSize',11);
legend('Location','northwest','FontSize',8);
xlim([1e-1 1e2]);

sgtitle('Passive Suspension - Influence of Damping Ratio h (Front Axle)');

% Console: cs values corresponding to each h
fprintf('\n=== h SWEEP — cs VALUES (Front Axle) ===\n');
fprintf('%-8s  %12s  %10s  %10s\n', 'h', 'cs [Ns/m]', 'zeta_I', 'zeta_II');
fprintf('%s\n', repmat('-',1,45));
for ih = 1:n_h
    cs_h = h_vec(ih) * cc1_F;
    fprintf('%-8.2f  %12.1f  %10.3f  %10.3f\n', ...
        h_vec(ih), cs_h, cs_h/cc1_F, cs_h/cc2_F);
end

%--- Final summary table --------------------------------------------------
fprintf('\n=== PEAK FRF AT BEST cs ===\n');
fprintf('%-32s  %12s  %12s\n', 'Metric', 'Front', 'Rear');
fprintf('%s\n', repmat('-',1,60));
fprintf('%-32s  %12.4f  %12.4f\n', 'DI ISO weighted peak [(m/s^2)/m]', ...
    peak_DI_F(idx_best_F), peak_DI_R(idx_best_R));
fprintf('%-32s  %12.2f  %12.2f\n', 'Road Holding peak [N/m]', ...
    peak_RH_F(idx_best_F), peak_RH_R(idx_best_R));
fprintf('%-32s  %12.5f  %12.5f\n', 'Working Space peak [m/m]', ...
    peak_WS_F(idx_best_F), peak_WS_R(idx_best_R));
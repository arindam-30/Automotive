%% =========================================================================
%  VERTICAL DYNAMICS - Passive / Semi-active / Active Suspension
%  Change ONLY the two lines marked <<<
%% =========================================================================

close all; clear; clc;

g = 9.81;
data_vehicle;
data_tire;
data_motor;
data_brake;
data_aero;
motor_working_points;
brake_distribution;
control_struct;

%--------------------------------------------------------------------------
% Suspension & vehicle parameters
%--------------------------------------------------------------------------
susp.muF = 35;    % [kg]  front unsprung mass
susp.muR = 30;    % [kg]  rear unsprung mass
susp.kF  = 20000; % [N/m] front spring stiffness
susp.kR  = 18000; % [N/m] rear spring stiffness

l  = veh.lF + veh.lR;
lF = veh.lF;
lR = veh.lR;
m  = veh.m;
kt = tir.kz;
ct = tir.cz;

mu = susp.muF;
ms = m * lR / l - mu;
ks = susp.kF;
cs = 5000;          % [Ns/m]

%--------------------------------------------------------------------------
% <<<  USER SETTINGS — CHANGE ONLY THESE TWO  >>>
%--------------------------------------------------------------------------
ctrl.mode = 1;      % 0 = passive | 1 = semi-active | 2 = active   <<<

road_files  = {'Road_Class_C_1000m.mat', 'Road_Class_A_1000m.mat'}; % <<<
road_labels = {'Class C', 'Class A'};
%--------------------------------------------------------------------------

% Control gains (derived from cs and ms — do not change)
ctrl.c_pas = cs / 2;
ctrl.c_sky = cs;
ctrl.b_sky = ctrl.c_pas/0.1;

mode_labels = {'Passive', 'Semi-active', 'Active'};
mode_str    = mode_labels{ctrl.mode + 1};

%--------------------------------------------------------------------------
% Simulation settings
%--------------------------------------------------------------------------
simu.dt      = 1e-4;
simu.dt_save = 1e-3;
simu.IC      = [0, 0, 0, 0];

V_kmh = [30, 50, 70, 90, 110, 130];
V_ms  = V_kmh / 3.6;
nV    = length(V_ms);
nR    = length(road_files);

%--------------------------------------------------------------------------
% Pre-allocate
%--------------------------------------------------------------------------
rms_comfort  = zeros(nR, nV);  max_comfort  = zeros(nR, nV);  min_comfort  = zeros(nR, nV);
rms_roadhold = zeros(nR, nV);  max_roadhold = zeros(nR, nV);  min_roadhold = zeros(nR, nV);
rms_worksp   = zeros(nR, nV);  max_worksp   = zeros(nR, nV);  min_worksp   = zeros(nR, nV);
rms_power    = zeros(nR, nV);  max_power    = zeros(nR, nV);
min_power    = zeros(nR, nV);  mean_power   = zeros(nR, nV);

all_rel_vel = [];
all_zsd     = [];
all_Fs      = [];

%==========================================================================
%% SIMULATION LOOP
%==========================================================================
for ir = 1:nR

    cd('RoadData\');
    load(road_files{ir});
    cd('..');

    for iv = 1:nV

        fprintf('[%s] Road: %-8s | Speed: %3d km/h\n', ...
            mode_str, road_labels{ir}, V_kmh(iv));

        simu.V     = V_ms(iv);
        simu.t_end = road.s(end) / simu.V;

        out = sim('Vertical_model.slx',             ...
                  'StopTime',  num2str(simu.t_end), ...
                  'FixedStep', num2str(simu.dt),    ...
                  'SaveTime',  'on',                ...
                  'TimeSaveName', 'tout');

        % --- Extract signals ---
        zsdd_sig = out.zsdd.Data;
        zs_sig   = out.zs.Data;
        zu_sig   = out.zu.Data;
        zsd_sig  = out.zsd.Data;
        zud_sig  = out.zud.Data;
        zr_sig   = out.zr.Data;
        zrd_sig  = out.zrd.Data;
        Fs_sig   = out.Fs.Data;

        % --- Derived ---
        tyre_F   = kt .* (zu_sig - zr_sig) + ct .* (zud_sig - zrd_sig);
        susp_def = (zs_sig - zu_sig) * 1e3;
        rel_vel  = zsd_sig - zud_sig;
        P_t      = Fs_sig .* rel_vel;

        % --- Comfort ---
        rms_comfort(ir,iv) = rms(zsdd_sig);
        max_comfort(ir,iv) = max(zsdd_sig);
        min_comfort(ir,iv) = min(zsdd_sig);

        % --- Road holding ---
        rms_roadhold(ir,iv) = rms(tyre_F);
        max_roadhold(ir,iv) = max(tyre_F);
        min_roadhold(ir,iv) = min(tyre_F);

        % --- Working space ---
        rms_worksp(ir,iv) = rms(susp_def);
        max_worksp(ir,iv) = max(susp_def);
        min_worksp(ir,iv) = min(susp_def);

        % --- Power ---
        rms_power(ir,iv)  = rms(P_t);
        max_power(ir,iv)  = max(P_t);
        min_power(ir,iv)  = min(P_t);
        mean_power(ir,iv) = mean(P_t);

        % --- Accumulate for surface (subsampled) ---
        skip = 10;
        all_rel_vel = [all_rel_vel; rel_vel(1:skip:end)];
        all_zsd     = [all_zsd;     zsd_sig(1:skip:end)];
        all_Fs      = [all_Fs;      Fs_sig(1:skip:end)];

    end
end

%==========================================================================
%% CONSOLE OUTPUT
%==========================================================================
fprintf('\n========== RESULTS (%s) ==========\n', mode_str);
for ir = 1:nR
    fprintf('\n--- Road: %s ---\n', road_labels{ir});
    fprintf('%-10s | %-10s %-10s %-10s | %-10s %-10s %-10s | %-10s %-10s %-10s\n', ...
        'Speed', ...
        'RMS zsdd','Max zsdd','Min zsdd', ...
        'RMS Ft',  'Max Ft',  'Min Ft',  ...
        'RMS WS',  'Max WS',  'Min WS');
    fprintf('%s\n', repmat('-',1,110));
    for iv = 1:nV
        fprintf('%-10s | %-10.4f %-10.4f %-10.4f | %-10.2f %-10.2f %-10.2f | %-10.3f %-10.3f %-10.3f\n', ...
            [num2str(V_kmh(iv)) ' km/h'], ...
            rms_comfort(ir,iv),  max_comfort(ir,iv),  min_comfort(ir,iv), ...
            rms_roadhold(ir,iv), max_roadhold(ir,iv), min_roadhold(ir,iv), ...
            rms_worksp(ir,iv),   max_worksp(ir,iv),   min_worksp(ir,iv));
    end
end

fprintf('\n========== POWER (%s) ==========\n', mode_str);
for ir = 1:nR
    fprintf('\n--- Road: %s ---\n', road_labels{ir});
    fprintf('%-10s | %-12s %-14s %-14s %-14s\n', ...
        'Speed','RMS P [W]','Mean P [W]','Max P [W]','Min P [W]');
    fprintf('%s\n', repmat('-',1,70));
    for iv = 1:nV
        fprintf('%-10s | %-12.2f %-14.2f %-14.2f %-14.2f\n', ...
            [num2str(V_kmh(iv)) ' km/h'], ...
            rms_power(ir,iv), mean_power(ir,iv), ...
            max_power(ir,iv), min_power(ir,iv));
    end
end

switch ctrl.mode
    case 0;  fprintf('\nNOTE: Passive — P >= 0 always.\n');
    case 1;  fprintf('\nNOTE: Semi-active — P >= 0 always (clipped to dissipative quadrants).\n');
    case 2;  fprintf('\nNOTE: Active — Mean P < 0 means actuator introduces net energy.\n');
end

%==========================================================================
%% PLOTS
%==========================================================================
lc = struct( ...
    'rms',  [0.00 0.00 0.00], ...
    'mx',   [0.85 0.33 0.10], ...
    'mn',   [0.93 0.69 0.13], ...
    'mean', [0.00 0.45 0.74]);

%--- Figure 1: Comfort Index ----------------------------------------------
figure('Name','Comfort Index','NumberTitle','off');
for ir = 1:nR
    subplot(nR,1,ir); hold on; grid on;
    plot(V_kmh, rms_comfort(ir,:),'-','Color',lc.rms,'LineWidth',1.8,'DisplayName','CI rms');
    plot(V_kmh, max_comfort(ir,:),'-','Color',lc.mx, 'LineWidth',1.8,'DisplayName','CI max');
    plot(V_kmh, min_comfort(ir,:),'-','Color',lc.mn, 'LineWidth',1.8,'DisplayName','CI min');
    ylabel('CI [m/s^2]'); title(road_labels{ir});
    legend('Location','best'); xlim([V_kmh(1) V_kmh(end)]);
end
xlabel('v [km/h]');
sgtitle(sprintf('Comfort Index — %s', mode_str));

%--- Figure 2: Road Holding -----------------------------------------------
figure('Name','Road Holding','NumberTitle','off');
for ir = 1:nR
    subplot(nR,1,ir); hold on; grid on;
    plot(V_kmh, rms_roadhold(ir,:),'-','Color',lc.rms,'LineWidth',1.8,'DisplayName','RH rms');
    plot(V_kmh, max_roadhold(ir,:),'-','Color',lc.mx, 'LineWidth',1.8,'DisplayName','RH max');
    plot(V_kmh, min_roadhold(ir,:),'-','Color',lc.mn, 'LineWidth',1.8,'DisplayName','RH min');
    ylabel('RH [N]'); title(road_labels{ir});
    legend('Location','best'); xlim([V_kmh(1) V_kmh(end)]);
end
xlabel('v [km/h]');
sgtitle(sprintf('Road Holding — %s', mode_str));

%--- Figure 3: Working Space ----------------------------------------------
figure('Name','Working Space','NumberTitle','off');
for ir = 1:nR
    subplot(nR,1,ir); hold on; grid on;
    plot(V_kmh, rms_worksp(ir,:),'-','Color',lc.rms,'LineWidth',1.8,'DisplayName','WS rms');
    plot(V_kmh, max_worksp(ir,:),'-','Color',lc.mx, 'LineWidth',1.8,'DisplayName','WS max');
    plot(V_kmh, min_worksp(ir,:),'-','Color',lc.mn, 'LineWidth',1.8,'DisplayName','WS min');
    ylabel('WS [mm]'); title(road_labels{ir});
    legend('Location','best'); xlim([V_kmh(1) V_kmh(end)]);
end
xlabel('v [km/h]');
sgtitle(sprintf('Working Space — %s', mode_str));

%--- Figure 4: Power ------------------------------------------------------
figure('Name','Suspension Power','NumberTitle','off');
for ir = 1:nR
    subplot(nR,1,ir); hold on; grid on;
    plot(V_kmh, rms_power(ir,:), '-','Color',lc.rms, 'LineWidth',1.8,'DisplayName','P rms');
    plot(V_kmh, mean_power(ir,:),'-','Color',lc.mean,'LineWidth',1.8,'DisplayName','P mean');
    plot(V_kmh, max_power(ir,:), '-','Color',lc.mx,  'LineWidth',1.8,'DisplayName','P max');
    plot(V_kmh, min_power(ir,:), '-','Color',lc.mn,  'LineWidth',1.8,'DisplayName','P min');
    yline(0,'k--','LineWidth',1.0,'HandleVisibility','off');
    ylabel('P [W]'); title(road_labels{ir});
    legend('Location','best'); xlim([V_kmh(1) V_kmh(end)]);
end
xlabel('v [km/h]');
sgtitle(sprintf('Suspension Power — %s', mode_str));

%--- Figures 5 & 6: Force and Power surfaces ------------------------------
n_grid = 80;   % denser grid for smoother curved surface (semi-active)
rv_lim = max(abs(all_rel_vel)) * 0.95;
zs_lim = max(abs(all_zsd))     * 0.95;
rv_vec = linspace(-rv_lim, rv_lim, n_grid);
zs_vec = linspace(-zs_lim, zs_lim, n_grid);
[RV, ZS] = meshgrid(rv_vec, zs_vec);

switch ctrl.mode
    case 0
        F_surf  = ctrl.c_pas .* RV;
        law_str = 'F = c_{pas}(\dot{z}_s - \dot{z}_u)';
    case 1
        % u = -b_sky*|zsd|*(zsd - zud)  when zsd*(zsd-zud) > 0, else 0
        % This gives a curved bilinear surface — different slope per |zsd| level
        F_surf       = zeros(size(RV));
        diss         = (ZS .* RV) > 0;
        F_surf(diss) = -ctrl.b_sky .* abs(ZS(diss)) .* RV(diss);
        law_str = 'u = -b_{sky}|\dot{z}_s|(\dot{z}_s-\dot{z}_u)\;[\dot{z}_s(\dot{z}_s-\dot{z}_u)>0],\;0\;\mathrm{otherwise}';
    case 2
        F_surf  = -ctrl.c_pas .* RV - ctrl.c_sky .* ZS;
        law_str = 'F = -c_s(\dot{z}_s-\dot{z}_u) - c_{sky}\dot{z}_s';
end

P_surf = F_surf .* RV;
cmap   = jet(n_grid);

% Axis labels as plain strings (no LaTeX interpreter issues)
xlab_rv = 'dz_s/dt - dz_u/dt  [m/s]';
ylab_zs = 'dz_s/dt  [m/s]';
cb_str  = 'dz_s/dt [m/s]';

% Figure 5 — Force surface
figure('Name',sprintf('Force Surface - %s',mode_str),'NumberTitle','off');

% 3D: meshgrid is (rv x zs) so surf(ZS, RV, F) puts:
%   X-axis (left plane)  = dz_s/dt
%   Y-axis (right plane) = dz_s/dt - dz_u/dt   <- matches slide
subplot(2,1,1);
surf(ZS, RV, F_surf,'EdgeColor','none'); colormap(jet); shading interp;
colorbar; grid on; view(-135, 25);
xlabel(ylab_zs,  'FontSize',11);          % left plane  = zsd
ylabel(xlab_rv,  'FontSize',11);          % right plane = relative vel
zlabel('F [N]',  'FontSize',11);
title(['$' law_str '$'],'Interpreter','latex','FontSize',12);

% 2D: F vs relative velocity, coloured by zsd level
subplot(2,1,2); hold on; grid on;
for ig = 1:n_grid
    plot(rv_vec, F_surf(ig,:),'-','Color',cmap(ig,:),'LineWidth',0.8);
end
xline(0,'k--','HandleVisibility','off');
yline(0,'k--','HandleVisibility','off');
xlabel(xlab_rv,'FontSize',12);
ylabel('F [N]','FontSize',12);
title(sprintf('Force vs Relative Velocity  (%s)  |  colour = dz_s/dt level', mode_str));
cb = colorbar;
cb.Label.String = cb_str;
clim([zs_vec(1) zs_vec(end)]);
sgtitle(sprintf('Force Characteristic — %s', mode_str));

% Figure 6 — Power surface
figure('Name',sprintf('Power Surface - %s',mode_str),'NumberTitle','off');

subplot(2,1,1);
surf(ZS, RV, P_surf,'EdgeColor','none'); colormap(jet); shading interp;
colorbar; grid on; view(-45,30);
hold on;
[ZSp, RVp] = meshgrid(linspace(-zs_lim,zs_lim,10), linspace(-rv_lim,rv_lim,10));
surf(ZSp, RVp, zeros(size(ZSp)),'FaceAlpha',0.15,'FaceColor','k','EdgeColor','none');
xlabel(ylab_zs, 'FontSize',11);
ylabel(xlab_rv, 'FontSize',11);
zlabel('P [W]', 'FontSize',11);
title(sprintf('Power Surface — %s', mode_str),'FontSize',12);

subplot(2,1,2); hold on; grid on;
for ig = 1:n_grid
    plot(rv_vec, P_surf(ig,:),'-','Color',cmap(ig,:),'LineWidth',0.8);
end
yline(0,'k--','LineWidth',1.2,'HandleVisibility','off');
xlabel(xlab_rv,'FontSize',12);
ylabel('P [W]','FontSize',12);
title(sprintf('Power vs Relative Velocity  (%s)  |  colour = dz_s/dt level', mode_str));
cb2 = colorbar;
cb2.Label.String = cb_str;
clim([zs_vec(1) zs_vec(end)]);
sgtitle(sprintf('Power Characteristic — %s', mode_str));
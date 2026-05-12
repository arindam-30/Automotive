%% Motor Working Points Analysis

% Motor force curve [N]
Fm = @(V) interp1(mot.vx, mot.Fm, V);

% Resistive forces
Fd     = @(V) 0.5 * aero.Cd * aero.rho * aero.A * V.^2;
Froll  = @(alpha) tir.Re * tir.fv * veh.m * g * cos(alpha);
Fslope = @(alpha) veh.m * g * sin(alpha);
Fres   = @(V, alpha) Fd(V) + Froll(alpha) + Fslope(alpha);

% Maximum speed on flat road
vx_max_flat = fsolve(@(x) Fm(x) - Fres(x, 0), 150/3.6);

% Road slopes and vehicle speeds
alpha_deg = [0, 5, 10];
alpha = deg2rad(alpha_deg);
V_kmh = [50, 70, 90, 110, 130, 150, vx_max_flat*3.6];

% Calculate V_max for each road slope
vx_max_per_alpha = zeros(length(alpha), 1);
for jj = 1:length(alpha)
    v0 = 100/3.6;
    [vx_max_sol, ~, exitflag] = fsolve(@(x) Fm(x) - Fres(x, alpha(jj)), v0);
    vx_max_per_alpha(jj) = vx_max_sol;
end

% Maximum velocity on flat road (alpha = 0) for Simulink model
vx_max = vx_max_per_alpha(1);

% Calculate gamma (motor admission grade) for each V and alpha
gamma_operating = NaN(length(V_kmh), length(alpha));
F_res_operating = NaN(length(V_kmh), length(alpha));

for ii = 1:length(V_kmh)
    V = V_kmh(ii) / 3.6;
    for jj = 1:length(alpha)
        if ~isnan(vx_max_per_alpha(jj)) && V <= vx_max_per_alpha(jj)
            F_res_operating(ii, jj) = Fres(V, alpha(jj));
            F_motor = Fm(V);
            if F_motor > 0
                gamma_operating(ii, jj) = F_res_operating(ii, jj) / F_motor;
            end
        end
    end
end

% Plot figures - show motor curves and resistive force, simplified legend
for jj = 1:length(alpha)
    figure;
    hold on; grid on;

    vx_plot = linspace(0, max(mot.vx), 500);

    % Plot resistive force curve
    Fres_curve = Fres(vx_plot, alpha(jj));
    plot(vx_plot * 3.6, Fres_curve * 1e-3, 'b-', 'LineWidth', 2, ...
        'DisplayName', 'F_{res}');

    % Plot motor curves for each operating gamma
    gamma_unique = unique(gamma_operating(:, jj));
    gamma_unique = gamma_unique(~isnan(gamma_unique));

    for k = 1:length(gamma_unique)
        gamma = gamma_unique(k);
        Fm_gamma = Fm(vx_plot) * gamma;
        valid = Fm_gamma > 0;
        plot(vx_plot(valid) * 3.6, Fm_gamma(valid) * 1e-3, 'LineWidth', 1.5, ...
            'DisplayName', sprintf('\\gamma = %.2f', gamma));
    end

    % Mark operating points (no legend entry)
    for ii = 1:length(V_kmh)
        if ~isnan(gamma_operating(ii, jj))
            F_at_V = F_res_operating(ii, jj);
            plot(V_kmh(ii), F_at_V * 1e-3, 'ro', 'MarkerSize', 8, ...
                'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
                'HandleVisibility', 'off');
        end
    end

    xlabel('V [km/h]');
    ylabel('F [kN]');
    title(sprintf('Motor Working Points - Road Slope \\alpha = %d^\\circ', alpha_deg(jj)));
    legend('show', 'Location', 'best');
    xlim([0 max(vx_plot)*3.6]);
end

% Display results table
fprintf('\n=== Maximum Velocity per Road Slope ===\n');
for jj = 1:length(alpha)
    fprintf('Alpha = %2d°: V_max = %7.1f km/h\n', alpha_deg(jj), vx_max_per_alpha(jj) * 3.6);
end

fprintf('\n=== Working Points Table ===\n');
for jj = 1:length(alpha)
    fprintf('--- Alpha = %d° (V_max = %.1f km/h) ---\n', alpha_deg(jj), vx_max_per_alpha(jj) * 3.6);
    fprintf('%8s\t%10s\t%10s\n', 'V [km/h]', 'F_x [N]', 'gamma [%]');
    fprintf('%8s\t%10s\t%10s\n', '---', '---', '---');

    for ii = 1:length(V_kmh)
        if ~isnan(gamma_operating(ii, jj))
            fprintf('%8.1f\t%10.1f\t%10.1f\n', ...
                V_kmh(ii), F_res_operating(ii, jj), gamma_operating(ii, jj) * 100);
        end
    end
    fprintf('\n');
end

% Export motor admission points to Excel
fprintf('Exporting motor admission points to Excel...\n');

% Prepare data for export
export_data = [];
for jj = 1:length(alpha)
    for ii = 1:length(V_kmh)
        if ~isnan(gamma_operating(ii, jj))
            export_data = [export_data; ...
                alpha_deg(jj), V_kmh(ii), F_res_operating(ii, jj), gamma_operating(ii, jj) * 100];
        end
    end
end

% Create table with headers
T = array2table(export_data, ...
    'VariableNames', {'Alpha_deg', 'V_kmh', 'Fx_N', 'gamma_percent'});

% Write to Excel file
excel_file = 'motor_admission_points.xlsx';
writetable(T, excel_file);
fprintf('Data exported to %s\n', excel_file);

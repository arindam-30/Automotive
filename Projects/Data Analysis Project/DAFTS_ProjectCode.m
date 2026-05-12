clc;
clear;

%Loading Individual Files
[filename, pathname] = uigetfile('*.mat', 'Select the MAT-file');
if isequal(filename, 0)
    disp('User selected Cancel');
else
    load(fullfile(pathname, filename));
end
%%SPATIAL LAG TESTS IN THE END(INDEPENDENT OF INDIVIDUAL DATA ANALYSIS)
ref  = 1;   % reference test
test = 4;   % test to compare against reference

%%ORIENTATION CORRECTION
if strcmp(filename, 'DA4FTS_Project#03_BlindChallenge#01_Test1.mat')
    Accelerometer_X = -Accelerometer_X;
    Accelerometer_Y = -Accelerometer_Y;
end
if strcmp(filename, 'DA4FTS_Project#03_BlindChallenge#01_Test3.mat')
    idx = 1:1:300;
    Accelerometer_X(idx) = Accelerometer_X(idx) * cosd(73.28);
end

%LONGITUDINAL ACCELERATION FILTERING
acc_x = Accelerometer_X;
acc_x = acc_x - mean(acc_x);
fs = 1/mean(diff(Time));
fc = 1.5; % Hz
[b,a] = butter(4, fc/(fs/2)); %4th order butterworth filter
acc_x = filtfilt(b,a,acc_x);  %zero phase filtering
figure;
plot(Time, Accelerometer_X,'r');
grid on;
hold on;
plot(Time, acc_x, 'b', 'LineWidth',2);
xlabel('Time');
ylabel('Longitudinal Acceleration');
title('Acc_x vs t');
legend('Raw Acc_X', 'Filtered Acc_x');

%%SPEED PROFILES
%Speed Profile using IMU data

velocity = cumtrapz(Time, acc_x); 
velocity_kmh = velocity * 3.6; 
[IMU_peak, IMU_idx]=max(abs(velocity_kmh));
%Speed Profile Using GPS
Latitude_deg  = Latitude./60;
Longitude_deg = Longitude./60;
t = Time;
fs = 1/mean(diff(t));
fc = 1.5;
[b,a] = butter(4,fc/(fs/2),'low');
Latitude_s  = filtfilt(b,a,Latitude_deg);
Longitude_s = filtfilt(b,a,Longitude_deg);
lat = deg2rad(Latitude_s);
lon = deg2rad(Longitude_s);
dt = diff(Time);
dt(dt <= 0) = NaN;     
dt = [dt(1); dt];     
R = 6371000;
dlat = diff(lat);  
dlon = diff(lon);
%Haversine Formula
a = sin(dlat/2).^2 + cos(lat(2:end)) .* cos(lat(1:end-1)) .* sin(dlon/2).^2;
c = 2 * atan2(sqrt(a), sqrt(1-a));
dist = [0; R * c];  
gps_speed = dist ./ dt;       
gps_speed = fillmissing(gps_speed, 'linear');
speed_kmh = gps_speed * 3.6;
speed1 = speed_kmh;
v = speed_kmh;
v_med = movmedian(v, 11);
res = v - v_med;
thr = 3 * mad(res,1); 
spikes = abs(res) > thr;
v_clean = v;
v_clean(spikes) = v_med(spikes);
[peak_v,loc_v] = max(abs(v_clean));
v_final = v_clean;
% Cross correlation of v_clean and velocity_kmh
[xcorr_v, lags_v] = xcorr(v_clean, velocity_kmh, 'coeff');
lag_time_v = lags_v / fs;
% Plot the cross-correlation
figure;
plot(lag_time_v, xcorr_v, 'k', 'LineWidth', 1.2);
xlabel('Lag [s]');
ylabel('Normalized Cross-Correlation');
title('Cross-Correlation between Cleaned Speed and GPS Speed');
grid on;
% Plot the Speed profile and identify max speed
figure;
plot(Time, v_final, 'b', 'LineWidth', 2.5);
hold on;
plot(Time(loc_v),v_final(loc_v),'bo','MarkerSize',2);
plot(Time,velocity_kmh,'g', 'LineWidth', 2.5);
plot(Time(IMU_idx),v_final(IMU_idx),'go','MarkerSize',2);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Corrected Speed vs Time');
grid on;
annotation('textbox', [0.15, 0.8, 0.1, 0.1], 'String', ['Max Speed(GPS) = ', num2str(peak_v) ' km/h'], 'FitBoxToText', 'on', 'BackgroundColor', 'white');
annotation('textbox', [0.15, 0.7, 0.1, 0.1], 'String', ['Max Speed(IMU) = ', num2str(IMU_peak) ' km/h'], 'FitBoxToText', 'on', 'BackgroundColor', 'white');

%%BRAKING DETECTION
dv = gradient(v_final, Time);
T_brake = 0.3;
acc_thr = -0.3;
dv_thr = -0.2;
brake_init = (acc_x < acc_thr) & (dv < dv_thr);
brake_init(1:loc_v) = false;
N_brake = round(T_brake * fs);
brake_mask = movsum(brake_init, N_brake) == N_brake;
brake_onset_idx = find(brake_mask, 1, 'first');
figure;
plot(Time, v_final, 'b', 'LineWidth', 1.5); hold on
grid on
xlabel('Time [s]')
ylabel('Speed [km/h]')
title('Velocity vs Time with Braking Onset')
xline(Time(loc_v), '--k', 'Peak Speed','FontSize',6);
if ~isempty(brake_onset_idx)
    t_brake = Time(brake_onset_idx);
    v_brake = v_final(brake_onset_idx);
    xline(t_brake, '--r', 'Braking Onset', 'FontSize', 6);
    plot(t_brake, v_brake, 'ro', 'MarkerFaceColor', 'r')
end
xlim([Time(loc_v) - 3, Time(loc_v) + 3]); 

%%STEERING CORRECTION ANALYSIS
%Filtering raw data (Bandpass Filter)
gyro_z = Gyroscope_Z - mean(Gyroscope_Z);
f_low = 1;
f_high = 4;
[b_gyro, a_gyro] = butter(4, [f_low, f_high]/(fs/2));
gyro_corr = filtfilt(b_gyro, a_gyro, gyro_z);
figure;
plot(Time,gyro_z,'b');
grid on;
hold on;
plot(Time,gyro_corr,'r');
xlabel('Time')
ylabel('Gyroscope Z data')
title('Raw vs Filtered Gyro_Z data')
legend('Raw Gyro Z', 'Filtered Gyro Z');
%PSD
N = length(gyro_corr);
[PSD,f] = pwelch(gyro_corr, hanning(N), [], [], fs);
figure
plot(f, PSD,'LineWidth',1.2)
xlim([0 6])
xlabel('Frequency [Hz]')
ylabel('PSD')
grid on
title('Frequency Content of Steering Corrections using PSD')
%FFT
N = length(gyro_corr);
w = hann(N);
Y = fft(gyro_corr .* w);
f = (0:N-1)*(fs/N);
Ymag = abs(Y)/N;
Ymag = Ymag(1:floor(N/2));
f = f(1:floor(N/2));
figure
plot(f, Ymag,'LineWidth',1.2)
xlim([0 6])
xlabel('Frequency [Hz]')
ylabel('Magnitude')
grid on
title('Frequency Content of Steering Correction using FFT')
%Corrections Detections
gyro_energy = gyro_corr.^2;
win_time = 0.25;
win_samples = round(win_time * fs);
energy_env = movsum(gyro_energy, win_samples);
thr_energy = 2 * rms(energy_env);
min_sep_samples = round(0.4 * fs);
[pksE, locsE] = findpeaks(energy_env,'MinPeakHeight', thr_energy, 'MinPeakDistance', min_sep_samples);
corr_time = Time(locsE);
corr_value = gyro_corr(locsE);
%Plotting
figure
subplot(2,1,1)
plot(Time, energy_env, 'k')
hold on
plot(corr_time, pksE, 'ro', 'LineWidth', 1.5)
grid on
ylabel('Correction Energy')
title('Correction Energy vs Time')
subplot(2,1,2)
plot(Time, gyro_corr, 'b')
hold on
plot(corr_time, corr_value, 'ro', 'LineWidth', 1.5)
grid on
xlabel('Time [s]')
ylabel('Filtered Gyro Z')
title('Major Steering Corrections')

%%MAX LATERAL ACCELERATION
t = Time;
ay_raw = Accelerometer_Y-mean(Accelerometer_Y);
yaw_raw = Gyroscope_Z-mean(Gyroscope_Z);
fc = 5;   % cutoff frequency (Hz)
[b,a]   = butter(4, fc/(fs/2));
ay_lp   = filtfilt(b, a, ay_raw);
yaw_lp  = filtfilt(b, a, yaw_raw);
window_time = 0.3;     
window_samples = round(window_time * fs);
ay_clean = sqrt(movmean(ay_lp.^2, window_samples));
[~, idx_corr] = max(abs(R));
[ay_max, idx] = max(ay_clean);
t_max = t(idx);
fprintf('Max lateral acceleration (clean): %.2f m/s^2\n', ay_max);
fprintf('Time: %.2f s\n', t_max);
figure
plot(t, ay_raw, 'Color', [0.7 0.7 0.7])
hold on
plot(t, ay_clean, 'b', 'LineWidth', 1.5)
plot(t(idx), ay_clean(idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2)
grid on
xlabel('Time (s)')
ylabel('Lateral Acceleration a_y (m/s^2)')
title('Max Lateral Acceleration')
legend('Raw', 'Cleaned', 'Max Lateral Acceleration');
annotation('textbox', [0.15, 0.8, 0.1, 0.1], 'String', ['Max acc_y = ', num2str(ay_max) 'm/s2'], 'FitBoxToText', 'on', 'BackgroundColor', 'white');

%Without moving RMS cleaning
ay = smoothdata(Accelerometer_Y,'movmean', 5);
fc= 2;
[b,a] = butter(4, fc/(fs/2));
acc_y = filtfilt(b,a,ay);
[p,loc] = max(abs(acc_y));
figure;
plot(Time,Accelerometer_Y,'Color',[0.7 0.7 0.7]); 
grid on;
hold on;
plot(Time(loc),acc_y(loc),'ro');
plot(Time, acc_y , 'b');
xlabel('Time(s)')
ylabel('Lateral Acceleration(m/s2)')
title('Lateral Acceleration vs Time')
annotation('textbox', [0.15, 0.8, 0.1, 0.1], 'String', ['Max acc_y = ', num2str(p) 'm/s2'], 'FitBoxToText', 'on', 'BackgroundColor', 'white');
[R_acc, lag] = xcorr(acc_y, ay_clean, 'coeff');
% Plot the correlation
figure;
plot(lag/fs, R_acc, 'LineWidth', 1.5);
grid on;
xlabel('Time Lag (s)');
ylabel('Correlation Coefficient');
title('Correlation between Cleaned Lateral Acceleration and Acceleration Data');

%%ROLL AND PITCH COMPARISON
gyro_x = Gyroscope_X;    % Roll rate
gyro_y = Gyroscope_Y;    % Pitch rate
gyro_x = gyro_x - mean(gyro_x);
gyro_y = gyro_y - mean(gyro_y);
fc = 5; 
[b_gyro_x, a_gyro_x] = butter(4, fc/(fs/2));
gyro_x_f = filtfilt(b_gyro_x, a_gyro_x, gyro_x); 
[b_gyro_y, a_gyro_y] = butter(4, fc/(fs/2));
gyro_y_f = filtfilt(b_gyro_y, a_gyro_y, gyro_y);
roll_int  = cumtrapz(t, gyro_x_f); %Integrating Gyro_X
pitch_int = cumtrapz(t, gyro_y_f); %Integrating Gyro_Y
roll_ref  = Roll;
pitch_ref = Pitch;
roll_ref  = roll_ref  - mean(roll_ref);
pitch_ref = pitch_ref - mean(pitch_ref);
fc = 5; 
roll_ref = lowpass(roll_ref, fc, fs); 
pitch_ref = lowpass(pitch_ref, fc, fs);

%Plotting
% Roll comparison
figure
plot(t, roll_ref, 'b', 'LineWidth', 1.2)
hold on
plot(t, roll_int, 'r--', 'LineWidth', 1.2)
grid on
xlabel('Time (s)')
ylabel('Roll Angle')
title('Roll Comparison')
legend('Measured Roll', 'Integrated Gyro X')

% Pitch comparison
figure
plot(t, pitch_ref, 'b', 'LineWidth', 1.2)
hold on
plot(t, pitch_int, 'r--', 'LineWidth', 1.2)
grid on
xlabel('Time (s)')
ylabel('Pitch Angle')
title('Pitch Comparison')
legend('Measured Pitch', 'Integrated Gyro Y')

%%DISPLAY ALL POINTS on GPS DATA
figure
plot(Longitude, Latitude, 'k','LineWidth',1.2)
hold on
plot(Longitude(locsE), Latitude(locsE),'ro','MarkerFaceColor','r','MarkerSize',6)
plot(Longitude(idx), Latitude(idx), 'b', 'Marker', '^', 'MarkerSize', 10);
plot(Longitude(brake_onset_idx), Latitude(brake_onset_idx), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 6);
plot(Longitude(loc_v), Latitude(loc_v), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'MarkerSize', 8)
axis equal
grid on
title('GPS Trajectory with Marked Data')
xlabel('Longitude')
ylabel('Latitude')
legend('GPS Trajectory', 'Steering Correction Points', 'Max Lateral Acceleration Point', 'Max Speed Point', 'Braking Onset Point'); 

files = {
    'DA4FTS_Project#03_BlindChallenge#01_Test1.mat'
    'DA4FTS_Project#03_BlindChallenge#01_Test2.mat'
    'DA4FTS_Project#03_BlindChallenge#01_Test3.mat'
    'DA4FTS_Project#03_BlindChallenge#01_Test4.mat'
};
N = numel(files);
data = cell(N,1);
for i = 1:N
    S = load(files{i});
    data{i} = S;   
end

function [v_clean, s] = compute_speed_profile(Latitude, Longitude, Time)
    Latitude_s  = smoothdata(Latitude,  'movmean', 5)./60;
    Longitude_s = smoothdata(Longitude, 'movmean', 5)./60;
    lat = deg2rad(Latitude_s);
    lon = deg2rad(Longitude_s);
    dt = diff(Time);
    dt(dt <= 0) = NaN;
    dt = [dt(1); dt];
    R = 6371000;
    dlat = diff(lat);
    dlon = diff(lon);
    a = sin(dlat/2).^2 +cos(lat(2:end)) .* cos(lat(1:end-1)) .* sin(dlon/2).^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    dist = [0; R * c];  
    gps_speed = dist ./ dt;
    gps_speed = fillmissing(gps_speed, 'linear');
    speed_kmh = gps_speed * 3.6;
    speed_kmh = smoothdata(speed_kmh, 'movmedian', 5);
    v = speed_kmh;
    v_med = movmedian(v, 11);
    res = v - v_med;
    thr = 3 * mad(res,1);
    spikes = abs(res) > thr;
    v_clean = v;
    v_clean(spikes) = v_med(spikes);
    s = cumsum(dist);
end

%VELOCITY PROFILES FOR ALL TESTS USING GPS DATA
v = cell(N,1);
s = cell(N,1);
for i = 1:N
    [v{i}, s{i}] = compute_speed_profile( ...
        data{i}.Latitude, ...
        data{i}.Longitude, ...
        data{i}.Time );
    fs = 1/mean(diff(data{i}.Time));
    fc = 2; % Hz
    [b,a] = butter(4, fc/(fs/2));
    v{i} = filtfilt(b,a,v{i});
end
figure;
plot(data{1}.Time, v{1}, 'b');
hold on;
grid on;
plot(data{2}.Time, v{2}, 'r');
plot(data{3}.Time, v{3}, 'g'); 
plot(data{4}.Time, v{4}, 'm'); 
xlabel('Time');
ylabel('Speed (km/h)');
title('Speed Profiles for All Tests');
legend('Test 1', 'Test 2', 'Test 3', 'Test 4');

%%SPATIAL LAG IDENTIFICATION
[~, iref]  = max(v{ref});
[~, itest] = max(v{test});
s_ref_max  = s{ref}(iref);
s_test_max = s{test}(itest);
window = 150;
mask_ref = s{ref} >= (s_ref_max - window) & ...
           s{ref} <= (s_ref_max + window);
s_ref_loc = s{ref}(mask_ref);
v_ref_loc = v{ref}(mask_ref);
v_ref_loc = v_ref_loc - mean(v_ref_loc,'omitnan');
mask_test = s{test} >= (s_test_max - window) & ...
            s{test} <= (s_test_max + window);
s_test_loc = s{test}(mask_test);
v_test_loc = v{test}(mask_test);
v_test_loc = v_test_loc - mean(v_test_loc,'omitnan');
v_test_interp = interp1( ...
    s_test_loc, v_test_loc, s_ref_loc, ...
    'linear','extrap');
[c, lags] = xcorr(v_test_interp, v_ref_loc, 'coeff');
ds = mean(diff(s_ref_loc));
lags_m = lags * ds;
[~, idx] = max(c);
spatial_lag = lags_m(idx);
figure; hold on; grid on
plot(lags_m, c, 'k', 'LineWidth', 1.5)
plot(spatial_lag, c(idx), 'ro', 'MarkerFaceColor','r')
xline(0, ':k', 'LineWidth', 1)
xline(spatial_lag, '--r', 'LineWidth', 1.2)
y_arrow = 0.95 * c(idx);
plot([0 spatial_lag], [y_arrow y_arrow], 'r', 'LineWidth', 2)
plot(0, y_arrow, '<r', 'MarkerFaceColor','r', 'MarkerSize',8)
plot(spatial_lag, y_arrow, 'r>', 'MarkerFaceColor','r', 'MarkerSize',8)
annotation('textbox', [0.15, 0.8, 0.1, 0.1], 'String', sprintf('Spatial Lag: %.2f m', spatial_lag), 'FitBoxToText', 'on', 'BackgroundColor', 'w');
xlabel('Space lag [m]')
ylabel('Cross-correlation')
title(sprintf('Spatial Lag at Max Speed: Test %d vs Test %d', test, ref))


function quarter_car_model()
% =========================================================
%   QUARTER CAR MODEL  --  Vehicle Dynamics (First Principles)
%   Interactive viewer: slider scrubber, Play/Pause, speed
%   Author : Vehicle Dynamics Engineer  |  Date : 2026
%
%   HOW TO RUN: type   quarter_car_model   in the Command Window
%               or press the green Run button.
%
%   System schematic (vertical, positive upward):
%
%         [  ms  ]  <- Sprung mass (car body)
%            |  |
%           Ks  Cs   <- Suspension spring & damper
%            |  |
%         [  mu  ]  <- Unsprung mass (wheel / tyre)
%            |  |
%           Kt  Ct   <- Tyre stiffness & structural damping
%            |  |
%          ======   <- Road input  z_r(t)
%
%   EOM (Newton 2nd Law, free-body diagram):
%     ms*z_s'' = -Ks*(zs-zu) - Cs*(zs'-zu')
%     mu*z_u'' =  Ks*(zs-zu) + Cs*(zs'-zu')
%               - Kt*(zu-zr) - Ct*(zu'-zr')
%
%   State : X = [zs, zs', zu, zu']'     Input : u = [zr, zr']'
% =========================================================

clc; close all;

% =========================================================
%  1. VEHICLE PARAMETERS
% =========================================================
ms = 320;      % [kg]      Sprung mass   (1/4 of 1280 kg car)
mu = 45;       % [kg]      Unsprung mass (wheel + hub + brake)
Ks = 22000;    % [N/m]     Suspension spring stiffness
Cs = 1500;     % [N.s/m]   Suspension damper
Kt = 190000;   % [N/m]     Tyre radial stiffness
Ct = 50;       % [N.s/m]   Tyre structural damping
g  = 9.81;     % [m/s^2]   Gravity

% =========================================================
%  2. ROAD PROFILE  (haversine bump)
% =========================================================
v_car  = 30/3.6;  % [m/s]   30 km/h forward speed
L_bump = 0.10;    % [m]     Bump half-width
H_bump = 0.08;    % [m]     Bump height (8 cm)
x_bump = 5.0;     % [m]     Distance from start to bump

t_end = 3.0;  dt = 0.001;
t     = 0:dt:t_end;
N     = length(t);
x_car = v_car * t;

z_r     = zeros(1,N);
z_r_dot = zeros(1,N);
for k = 1:N
    xi = x_car(k) - x_bump;
    if xi >= 0 && xi <= 2*L_bump
        z_r(k)     = H_bump * sin(pi*xi/(2*L_bump))^2;
        dzdx        = H_bump * sin(pi*xi/(2*L_bump)) ...
                      * cos(pi*xi/(2*L_bump)) * pi/L_bump;
        z_r_dot(k) = dzdx * v_car;
    end
end

% =========================================================
%  3. STATE-SPACE MATRICES
% =========================================================
A = [  0,       1,          0,           0;
      -Ks/ms, -Cs/ms,     Ks/ms,       Cs/ms;
       0,       0,          0,           1;
       Ks/mu,  Cs/mu,  -(Ks+Kt)/mu, -(Cs+Ct)/mu ];

B = [  0,      0;
       0,      0;
       0,      0;
       Kt/mu, Ct/mu ];

% =========================================================
%  4. RK4 NUMERICAL INTEGRATION
% =========================================================
X = zeros(4,N);
for k = 1:N-1
    ui   = [z_r(k);   z_r_dot(k)  ];
    uip1 = [z_r(k+1); z_r_dot(k+1)];
    umid = 0.5*(ui+uip1);
    f1 = A*X(:,k)            + B*ui;
    f2 = A*(X(:,k)+dt/2*f1) + B*umid;
    f3 = A*(X(:,k)+dt/2*f2) + B*umid;
    f4 = A*(X(:,k)+dt*f3)   + B*uip1;
    X(:,k+1) = X(:,k) + (dt/6)*(f1+2*f2+2*f3+f4);
end

z_s_abs     = X(1,:);
z_u_abs     = X(3,:);
susp_travel = X(1,:) - X(3,:);
z_s_ddot    = (A(2,:)*X + B(2,:)*[z_r; z_r_dot]) / g;

fprintf('RK4 done. Building interactive window...\n');

% =========================================================
%  5. COLOUR PALETTE & GEOMETRY  (shared by nested fns)
% =========================================================
cBody   = [0.20 0.60 1.00];
cWheel  = [0.95 0.70 0.10];
cRoad   = [0.45 0.45 0.50];
cSpring = [0.30 0.95 0.50];
cDamper = [1.00 0.45 0.20];
cBg     = [0.10 0.10 0.14];
cAxBg   = [0.15 0.15 0.21];
cGrid   = [0.25 0.25 0.33];
cCtrl   = [0.13 0.13 0.20];
cText   = [0.90 0.90 0.95];
cBtn    = [0.22 0.22 0.32];

R_wheel    = 0.30;
car_W      = 1.20;  car_H = 0.40;
scale      = 6;            % amplify small displacements for visibility
y_road_ref = 0.0;
y_axle_ref = y_road_ref + R_wheel;
y_body_ref = y_axle_ref + 0.45;
n_coils    = 7;
theta_c    = linspace(0,2*pi,80);
car_xv     = [-car_W/2, car_W/2, car_W/2, -car_W/2, -car_W/2];
car_yv     = [0, 0, car_H, car_H, 0];
ws_x       = [-car_W/2+0.15, car_W/2-0.15, car_W/2-0.30, -car_W/2+0.30];
ws_y       = [car_H-0.02,    car_H-0.02,   car_H+0.22,   car_H+0.22   ];

% =========================================================
%  6. FIGURE & AXES
% =========================================================
fig = figure('Name','Quarter Car Model  --  Interactive', ...
    'Color',cBg, 'Position',[60 40 1360 820], 'Resize','off');

% -- Animation panel (left 40%)
ax_anim = axes('Parent',fig, ...
    'Position',[0.01 0.10 0.40 0.88], ...
    'Color',cAxBg,'XColor','none','YColor','none', ...
    'XLim',[-1.65 1.65],'YLim',[-0.40 2.15], ...
    'DataAspectRatio',[1 1 1]);
hold(ax_anim,'on');
title(ax_anim,'Animation  (displacement x6 amplified)', ...
    'Color',cText,'FontSize',11,'FontWeight','bold');

% -- Signal axes (right 53%)
ax1 = axes('Parent',fig,'Position',[0.45 0.72 0.53 0.24],'Color',cAxBg);
ax2 = axes('Parent',fig,'Position',[0.45 0.42 0.53 0.24],'Color',cAxBg);
ax3 = axes('Parent',fig,'Position',[0.45 0.12 0.53 0.24],'Color',cAxBg);

for ax = [ax1 ax2 ax3]
    set(ax,'GridColor',cGrid,'GridAlpha',1,'XColor',cText,'YColor',cText, ...
           'FontSize',9,'Box','on','Color',cAxBg,'XLim',[0 t_end]);
    grid(ax,'on'); hold(ax,'on');
end
ylabel(ax1,'Disp. (m)','Color',cText);
title(ax1,'Body & Wheel Displacement','Color',cText,'FontWeight','bold');
ylabel(ax2,'Susp. travel (m)','Color',cText);
title(ax2,'Suspension Travel','Color',cText,'FontWeight','bold');
ylabel(ax3,'Accel. (g)','Color',cText);
title(ax3,'Body Vertical Acceleration','Color',cText,'FontWeight','bold');
xlabel(ax3,'Time (s)','Color',cText);

% Ghost traces (full run, faded background)
line(ax1,t,z_s_abs,    'Color',[cBody   0.18],'LineWidth',0.8);
line(ax1,t,z_u_abs,    'Color',[cWheel  0.18],'LineWidth',0.8);
line(ax1,t,z_r,        'Color',[cRoad   0.28],'LineWidth',0.8);
line(ax2,t,susp_travel,'Color',[cSpring 0.18],'LineWidth',0.8);
line(ax3,t,z_s_ddot,   'Color',[cDamper 0.18],'LineWidth',0.8);

% Live traces
ln1a = line(ax1,NaN,NaN,'Color',cBody,  'LineWidth',2.0,'DisplayName','Body  z_s');
ln1b = line(ax1,NaN,NaN,'Color',cWheel, 'LineWidth',2.0,'DisplayName','Wheel z_u');
ln1c = line(ax1,NaN,NaN,'Color',cRoad,  'LineWidth',1.4,'DisplayName','Road  z_r','LineStyle','--');
legend(ax1,[ln1a ln1b ln1c],'Location','northeast', ...
    'TextColor',cText,'Color',cAxBg,'EdgeColor',cGrid,'FontSize',8);
ln2 = line(ax2,NaN,NaN,'Color',cSpring,'LineWidth',2.0);
ln3 = line(ax3,NaN,NaN,'Color',cDamper,'LineWidth',2.0);

% Cursor lines (vertical bar tracking current time)
csr1 = xline(ax1,0,'Color',[cText 0.7],'LineWidth',1.0,'LineStyle',':');
csr2 = xline(ax2,0,'Color',[cText 0.7],'LineWidth',1.0,'LineStyle',':');
csr3 = xline(ax3,0,'Color',[cText 0.7],'LineWidth',1.0,'LineStyle',':');

% =========================================================
%  7. ANIMATION OBJECTS
% =========================================================
% Road slab
x_rd = linspace(-1.65,1.65,600);
patch(ax_anim,[x_rd, fliplr(x_rd)], ...
    [y_road_ref*ones(1,600), (y_road_ref-0.20)*ones(1,600)], ...
    cRoad,'EdgeColor','none','FaceAlpha',0.80);
% Lane markings
for xm = -1.5:0.45:1.5
    patch(ax_anim,[xm xm+0.20 xm+0.20 xm], ...
        [-0.015 -0.015 -0.040 -0.040],[1 1 1], ...
        'EdgeColor','none','FaceAlpha',0.22);
end

% Wheel
wheel_h = patch(ax_anim, R_wheel*cos(theta_c), ...
    R_wheel*sin(theta_c)+y_axle_ref, ...
    cWheel,'EdgeColor',cWheel*0.55,'LineWidth',1.5);
spoke_h = gobjects(5,1);
for s = 1:5
    ang = (s-1)*2*pi/5;
    spoke_h(s) = line(ax_anim, ...
        [0, R_wheel*0.82*cos(ang)], ...
        [y_axle_ref, y_axle_ref+R_wheel*0.82*sin(ang)], ...
        'Color',cWheel*0.50,'LineWidth',1.8);
end
hub_h = plot(ax_anim,0,y_axle_ref,'o','MarkerSize',7, ...
    'MarkerFaceColor','k','MarkerEdgeColor',cWheel*0.4);

% Car body, windscreen, roof
body_h = patch(ax_anim,car_xv,car_yv+y_body_ref, ...
    cBody,'EdgeColor',cBody*0.6,'LineWidth',1.5,'FaceAlpha',0.88);
windscreen_h = patch(ax_anim,ws_x,ws_y+y_body_ref, ...
    [0.50 0.78 1.00],'EdgeColor','none','FaceAlpha',0.55);
roof_h = patch(ax_anim, ...
    [-car_W/2+0.30, car_W/2-0.30, car_W/2-0.30, -car_W/2+0.30], ...
    [car_H+0.22, car_H+0.22, car_H+0.42, car_H+0.42]+y_body_ref, ...
    cBody*0.78,'EdgeColor','none','FaceAlpha',0.92);

% Suspension spring + damper
spring_h     = line(ax_anim,NaN,NaN,'Color',cSpring,'LineWidth',2.3);
damper_h     = line(ax_anim,NaN,NaN,'Color',cDamper,'LineWidth',4);
damper_box_h = patch(ax_anim,NaN,NaN,cDamper, ...
    'EdgeColor',cDamper*0.55,'FaceAlpha',0.68);

% Labels inside animation panel
t_label = text(ax_anim,0.02,0.97,'t = 0.000 s','Units','normalized', ...
    'Color',cText,'FontSize',11,'FontWeight','bold','VerticalAlignment','top');
text(ax_anim,0.02,0.91, ...
    sprintf('v = %.0f km/h  |  Bump = %.0f cm',v_car*3.6,H_bump*100), ...
    'Units','normalized','Color',cText,'FontSize',9);
text(ax_anim,0.02,0.87,'Car body',   'Units','normalized','Color',cBody,  'FontSize',9,'FontWeight','bold');
text(ax_anim,0.02,0.83,'Wheel',      'Units','normalized','Color',cWheel, 'FontSize',9,'FontWeight','bold');
text(ax_anim,0.02,0.79,'Suspension', 'Units','normalized','Color',cSpring,'FontSize',9,'FontWeight','bold');
text(ax_anim,0.02,0.75,'Damper',     'Units','normalized','Color',cDamper,'FontSize',9,'FontWeight','bold');
param_str = sprintf('ms=%g kg    mu=%g kg\nKs=%g N/m    Cs=%g N.s/m\nKt=%g N/m    Ct=%g N.s/m', ...
    ms,mu,Ks,Cs,Kt,Ct);
text(ax_anim,0.50,0.045,param_str,'Units','normalized','Color',cText*0.72, ...
    'FontSize',7.5,'HorizontalAlignment','center','VerticalAlignment','bottom', ...
    'BackgroundColor',cAxBg,'EdgeColor',cGrid,'Margin',3);

% =========================================================
%  8. CONTROL STRIP  (bottom 9.5% of figure)
% =========================================================
annotation(fig,'rectangle',[0 0 1 0.095], ...
    'Color','none','FaceColor',cCtrl);

% Labels
uicontrol(fig,'Style','text','String','TIME SCRUBBER', ...
    'Units','normalized','Position',[0.010 0.058 0.130 0.028], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.65, ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','left');
t_val_lbl = uicontrol(fig,'Style','text','String','t = 0.000 s', ...
    'Units','normalized','Position',[0.840 0.058 0.148 0.030], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText, ...
    'FontSize',10,'FontWeight','bold','HorizontalAlignment','right');
uicontrol(fig,'Style','text','String','0 s', ...
    'Units','normalized','Position',[0.010 0.006 0.038 0.026], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.55,'FontSize',8);
uicontrol(fig,'Style','text','String',sprintf('%.1f s',t_end), ...
    'Units','normalized','Position',[0.902 0.006 0.058 0.026], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.55,'FontSize',8);

% Main time slider
slider_h = uicontrol(fig,'Style','slider', ...
    'Units','normalized','Position',[0.010 0.006 0.895 0.048], ...
    'Min',1,'Max',N,'Value',1,'SliderStep',[1/(N-1), 50/(N-1)], ...
    'BackgroundColor',[0.28 0.52 0.88]);

% Play/Pause
play_btn = uicontrol(fig,'Style','pushbutton','String','  Play', ...
    'Units','normalized','Position',[0.205 0.010 0.082 0.072], ...
    'BackgroundColor',[0.12 0.62 0.28],'ForegroundColor','w', ...
    'FontSize',11,'FontWeight','bold');

% Reset
reset_btn = uicontrol(fig,'Style','pushbutton','String','Reset', ...
    'Units','normalized','Position',[0.295 0.010 0.062 0.072], ...
    'BackgroundColor',cBtn,'ForegroundColor',cText,'FontSize',10,'FontWeight','bold');

% Step buttons
stepb_btn = uicontrol(fig,'Style','pushbutton','String','|<', ...
    'Units','normalized','Position',[0.364 0.024 0.036 0.054], ...
    'BackgroundColor',cBtn,'ForegroundColor',cText,'FontSize',10,'FontWeight','bold');
stepf_btn = uicontrol(fig,'Style','pushbutton','String','>|', ...
    'Units','normalized','Position',[0.402 0.024 0.036 0.054], ...
    'BackgroundColor',cBtn,'ForegroundColor',cText,'FontSize',10,'FontWeight','bold');

% Loop toggle
loop_btn = uicontrol(fig,'Style','togglebutton','String','Loop ON', ...
    'Units','normalized','Position',[0.444 0.024 0.068 0.054], ...
    'BackgroundColor',[0.12 0.42 0.22],'ForegroundColor',cText, ...
    'FontSize',9,'FontWeight','bold','Value',1);

% Speed
uicontrol(fig,'Style','text','String','Speed', ...
    'Units','normalized','Position',[0.520 0.055 0.048 0.026], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.75,'FontSize',8);
spd_lbl = uicontrol(fig,'Style','text','String','1.0x', ...
    'Units','normalized','Position',[0.520 0.010 0.048 0.038], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText,'FontSize',10,'FontWeight','bold');
uicontrol(fig,'Style','text','String','0.25x', ...
    'Units','normalized','Position',[0.572 0.055 0.048 0.026], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.50,'FontSize',7.5);
uicontrol(fig,'Style','text','String','4x', ...
    'Units','normalized','Position',[0.754 0.055 0.028 0.026], ...
    'BackgroundColor',cCtrl,'ForegroundColor',cText*0.50,'FontSize',7.5);
spd_slider = uicontrol(fig,'Style','slider', ...
    'Units','normalized','Position',[0.572 0.018 0.214 0.040], ...
    'Min',0.25,'Max',4.0,'Value',1.0, ...
    'SliderStep',[0.25/(4-0.25), 1/(4-0.25)], ...
    'BackgroundColor',[0.40 0.40 0.55]);

% =========================================================
%  9. SHARED STATE on figure UserData
% =========================================================
state.frame   = 1;
state.playing = false;
fig.UserData  = state;

% =========================================================
%  10. RENDER  (nested function -- sees all parent variables)
% =========================================================
    function render(i)
        % Displacements amplified for visibility
        dy_u = z_u_abs(i) * scale;
        dy_s = z_s_abs(i) * scale;
        dy_r = z_r(i)     * scale;

        y_wc = y_axle_ref + dy_u + dy_r;  % wheel centre
        y_bb = y_body_ref + dy_s;          % body bottom

        % Wheel + rolling spokes
        set(wheel_h,'XData',R_wheel*cos(theta_c), ...
                    'YData',R_wheel*sin(theta_c)+y_wc);
        set(hub_h,'XData',0,'YData',y_wc);
        rot = t(i) * 8;
        for s = 1:5
            ang = (s-1)*2*pi/5 + rot;
            set(spoke_h(s), ...
                'XData',[0, R_wheel*0.82*cos(ang)], ...
                'YData',[y_wc, y_wc+R_wheel*0.82*sin(ang)]);
        end

        % Body, windscreen, roof
        set(body_h,      'YData', car_yv + y_bb);
        set(windscreen_h,'YData', ws_y   + y_bb);
        set(roof_h,'YData', ...
            [car_H+0.22,car_H+0.22,car_H+0.42,car_H+0.42]+y_bb);

        % Spring (left of suspension strut)
        y_sp_bot = y_wc + R_wheel*0.06;
        [xs_s,ys_s] = spring_coords(-0.28, y_sp_bot, y_bb, n_coils, 0.065);
        set(spring_h,'XData',xs_s,'YData',ys_s);

        % Damper (right of suspension strut)
        y_dp_bot = y_wc + R_wheel*0.06;
        y_dp_mid = (y_dp_bot + y_bb)/2;
        set(damper_h,'XData',[0.28 0.28],'YData',[y_dp_bot y_bb]);
        bw=0.072; bh=0.082;
        set(damper_box_h, ...
            'XData',[0.28-bw,0.28+bw,0.28+bw,0.28-bw], ...
            'YData',[y_dp_mid-bh,y_dp_mid-bh,y_dp_mid+bh,y_dp_mid+bh]);

        % Time label in animation panel
        set(t_label,'String',sprintf('t = %.3f s',t(i)));

        % Signal traces (history up to frame i)
        idx = 1:i;
        set(ln1a,'XData',t(idx),'YData',z_s_abs(idx));
        set(ln1b,'XData',t(idx),'YData',z_u_abs(idx));
        set(ln1c,'XData',t(idx),'YData',z_r(idx));
        set(ln2, 'XData',t(idx),'YData',susp_travel(idx));
        set(ln3, 'XData',t(idx),'YData',z_s_ddot(idx));

        % Cursor lines
        set(csr1,'Value',t(i));
        set(csr2,'Value',t(i));
        set(csr3,'Value',t(i));

        % Sync slider and readout
        set(slider_h,'Value',i);
        set(t_val_lbl,'String',sprintf('t = %.3f s',t(i)));
    end

% =========================================================
%  11. SPRING HELPER  (nested -- visible to render too)
% =========================================================
    function [xs,ys] = spring_coords(x0, y_bot, y_top, nc, amp)
        np = nc*4 + 2;
        ys = linspace(y_bot, y_top, np);
        xr = zeros(1,np);
        for k = 2:np-1
            xr(k) = amp * sin((k-1)*pi/2);
        end
        xs = x0 + xr;
    end

% =========================================================
%  12. TIMER  (~30 fps playback driver)
% =========================================================
tmr = timer('ExecutionMode','fixedRate','Period',0.033, ...
    'TimerFcn',@timer_tick);

    function timer_tick(~,~)
        if ~isvalid(fig), stop(tmr); delete(tmr); return; end
        st = fig.UserData;
        if ~st.playing, return; end

        spd      = get(spd_slider,'Value');
        st.frame = st.frame + max(1, round(spd*5));

        if st.frame >= N
            if get(loop_btn,'Value')
                st.frame = 1;          % loop back
            else
                st.frame   = N;
                st.playing = false;
                set(play_btn,'String','  Play', ...
                    'BackgroundColor',[0.12 0.62 0.28]);
            end
        end
        fig.UserData = st;
        render(st.frame);
        drawnow limitrate;
    end

% =========================================================
%  13. CALLBACKS  (all nested -- full variable access)
% =========================================================

    % -- Slider drag -----------------------------------------
    function slider_drag(src,~)
        if ~isvalid(fig), return; end
        i = max(1, min(N, round(get(src,'Value'))));
        st = fig.UserData;
        st.frame = i;
        fig.UserData = st;
        render(i);
        drawnow limitrate;
    end

    % -- Play / Pause ----------------------------------------
    function play_pause_cb(~,~)
        if ~isvalid(fig), return; end
        st = fig.UserData;
        st.playing = ~st.playing;
        if st.playing
            if st.frame >= N, st.frame = 1; end
            set(play_btn,'String',' Pause', ...
                'BackgroundColor',[0.72 0.25 0.10]);
            if strcmp(tmr.Running,'off'), start(tmr); end
        else
            set(play_btn,'String','  Play', ...
                'BackgroundColor',[0.12 0.62 0.28]);
        end
        fig.UserData = st;
    end

    % -- Reset -----------------------------------------------
    function reset_cb(~,~)
        if ~isvalid(fig), return; end
        st = fig.UserData;
        st.frame   = 1;
        st.playing = false;
        fig.UserData = st;
        set(play_btn,'String','  Play','BackgroundColor',[0.12 0.62 0.28]);
        render(1);
        drawnow;
    end

    % -- Step ±50 frames -------------------------------------
    function step_frame(delta)
        if ~isvalid(fig), return; end
        st = fig.UserData;
        st.frame = max(1, min(N, st.frame + delta));
        fig.UserData = st;
        render(st.frame);
        drawnow limitrate;
    end

    % -- Loop toggle label -----------------------------------
    function loop_toggle_cb(src,~)
        if get(src,'Value')
            set(src,'String','Loop ON','BackgroundColor',[0.12 0.42 0.22]);
        else
            set(src,'String','Loop OFF','BackgroundColor',cBtn);
        end
    end

    % -- Speed slider label ----------------------------------
    function spd_change(src,~)
        set(spd_lbl,'String',sprintf('%.2gx',get(src,'Value')));
    end

    % -- Cleanup on window close ----------------------------
    function cleanup_cb(~,~)
        if isvalid(tmr), stop(tmr); delete(tmr); end
        delete(fig);
    end

% =========================================================
%  14. WIRE UP CALLBACKS
% =========================================================
addlistener(slider_h,'ContinuousValueChange',@slider_drag);
set(play_btn,  'Callback', @play_pause_cb);
set(reset_btn, 'Callback', @reset_cb);
set(stepb_btn, 'Callback', @(~,~) step_frame(-50));
set(stepf_btn, 'Callback', @(~,~) step_frame(+50));
set(loop_btn,  'Callback', @loop_toggle_cb);
addlistener(spd_slider,'ContinuousValueChange',@spd_change);
set(fig,'CloseRequestFcn',@cleanup_cb);

% =========================================================
%  15. INITIAL RENDER + START TIMER
% =========================================================
render(1);        % draw frame 1 so window isn't blank
start(tmr);       % timer is idle until playing=true

% =========================================================
%  16. CONSOLE SUMMARY
% =========================================================
fprintf('\n============================================\n');
fprintf('  QUARTER CAR -- INTERACTIVE VIEWER READY  \n');
fprintf('============================================\n');
fprintf('  ms=%g kg     mu=%g kg\n',       ms,mu);
fprintf('  Ks=%g N/m    Cs=%g N.s/m\n',   Ks,Cs);
fprintf('  Kt=%g N/m    Ct=%g N.s/m\n',   Kt,Ct);
fprintf('  Speed=%.0f km/h  Bump=%.0fcm x %.0fcm\n', ...
        v_car*3.6, H_bump*100, L_bump*200);
fprintf('--------------------------------------------\n');
fprintf('  Peak body disp.   = %.4f m\n', max(abs(z_s_abs)));
fprintf('  Peak wheel disp.  = %.4f m\n', max(abs(z_u_abs)));
fprintf('  Peak susp travel  = %.4f m\n', max(abs(susp_travel)));
fprintf('  Peak body accel.  = %.4f g\n', max(abs(z_s_ddot)));
fprintf('============================================\n');
fprintf('  Drag slider      -> scrub to any instant  \n');
fprintf('  Play / Pause     -> run / freeze           \n');
fprintf('  Reset            -> back to t=0            \n');
fprintf('  |< / >|          -> +/-50 frames           \n');
fprintf('  Speed slider     -> 0.25x to 4x            \n');
fprintf('  Loop toggle      -> auto-rewind             \n');
fprintf('============================================\n');

end  % <-- closes the top-level function quarter_car_model()

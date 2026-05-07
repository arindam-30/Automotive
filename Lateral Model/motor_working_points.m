%--------------------------------------------------------------------------
% Define the function of motor force and resistance forces
%--------------------------------------------------------------------------

Fm = @(V) interp1(mot.vx,mot.Fm,V); % Definition of the motor force curve applied on the car

Fd     = @(V) 0.5 * aero.Cd * aero.rho * aero.A * V.^2; % Drag force
Froll  = @(alpha) tir.Re * tir.fv * veh.m*g * cos(alpha); % Rolling resistance force
Fslope = @(alpha) veh.m*g * sin(alpha); % Resistance force due to the road slope
Fres = @(V,alpha) Fd(V) + Froll(alpha) + Fslope(alpha); % Total resistance force applied on the car

%--------------------------------------------------------------------------
% Calculate the maximum speed of the car
%--------------------------------------------------------------------------
vx_max = fsolve(@(x) Fm(x)-Fres(x,0),150/3.6); % [m/s]

% Calculate the motor admission grade for different working point
V = [50, 70, 90, 110, 130, 150, vx_max/3.6]/3.6; % [m/s] Vehicle speed
alpha = deg2rad(0); % [rad] Road slope

vx = linspace(0,vx_max,1000);

figure;
gamma = NaN(length(V), length(alpha)); % initialise gamma with NaNs

for ii = 1:length(V)
    for jj = 1:length(alpha)
        gamma(ii,jj) = Fres( V(ii), alpha(jj) ) / Fm( V(ii) );
        if gamma(ii,jj)>1
            gamma(ii,jj) = NaN;
        end
    end
    plot(vx*3.6, Fm(vx)*gamma(ii,jj)*1e-3,'LineWidth',1); hold on;grid on;
end
plot(vx*3.6, Fres(vx,alpha)*1e-3,'b','LineWidth',2);
plot(mot.vx*3.6, mot.Fm*1e-3,'r','LineWidth',2);
xlabel('V [km/h]'); 
ylabel('F [kN]');

clear vx alpha V gamma  
%--------------------------------------------------------------------------
% Load brakes data
%--------------------------------------------------------------------------

bra.KbF = 16.54; % [Nm/bar] Torque to pressure ratio referred to a single wheel
bra.KbR = 7.49; % [Nm/bar] Torque to pressure ratio referred to a single wheel

bra.tau = [ 0.15, 4.15, 40 ];
bra.H = tf(bra.tau(3),bra.tau);
% figure()
% bode(bra.H)
% figure()
% step(bra.H)
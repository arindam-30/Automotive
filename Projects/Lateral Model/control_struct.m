cc.dt = 1e-2; 

cc.Kb = 2*(bra.KbF+bra.KbR);

cc.Re = 0.3;
cc.m = veh.m;
cc.fv = tir.fv;
cc.Froll = cc.fv * cc.m*g;
cc.Faero = 0.5*aero.Cd*aero.rho*aero.A;

cc.kp = 5000;
cc.Ti = 100;
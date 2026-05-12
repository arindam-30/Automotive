%--------------------------------------------------------------------------
% Load tire data
%--------------------------------------------------------------------------

% general dimensions
tir.kz   = 320000; % [N/m] tire vertical stiffness
tir.cz   = 50; % [N/m] tire vertical stiffness
tir.Fz0  = 4000; % [N] nominal tire load
tir.R0   = 0.326; % [m] unloaded radius
tir.Re   = 0.31948; % [m] effective rolling radius
tir.fv   = 0.01*2; % [-] axle rolling resistance coefficient

% Pacejka formula longitudinal coefficient
tir.Cx   = 1.839;
tir.Ex   = 0.6209973;
tir.pDx1 =1.36644;
tir.pDx2 = -0.11999;
tir.Dx   = tir.pDx1.*tir.Fz0;
tir.pKx1 = 18.886;
tir.pKx2 = -3.988;
tir.pKx3 = 0.21542;
tir.Kx   = tir.pKx1.*tir.Fz0;
tir.Bx   = tir.Kx/(tir.Cx*tir.Dx);

% Pacejka formula lateral coefficient
tir.Cy   =1.322298206 ;
tir.Ey   =-0.637732886 ;
tir.pDy1 =1.131735213 ;
tir.pDy2 =-0.147287597 ;
tir.Dy   = tir.pDy1.*tir.Fz0;
tir.pKy1 = 19.40101948;
tir.pKy2 =-1.799899788 ;
tir.pKy3 = 0.009539557;
tir.Ky   = tir.pKy1.*tir.Fz0;
tir.By   = tir.Ky/(tir.Cy*tir.Dy);

% relaxation lengths
tir.Lx   =0.03 ;
tir.Ly   = 0.38;
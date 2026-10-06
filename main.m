close all; clc; clear

global m mA mRV mRH g JRV JRH tpV tpH alpha_y lH lV l lAV lAH hS hA
global PhiA PhiB
global maxIter tol

% Simulation parameters
maxIter = 50;
tol     = 1e-7;

% Road gradient in %
qSt = 10;

% Vehicle parameters
mA  = 1800;                  % [kg]      Mass of the vehicle body
mRV  = 80;                  % [kg]      Mass of the front axle wheels
mRH  = 80;                  % [kg]      Mass of the rear axle wheels
g   = 9.81;                  % [m/s^2]   Gravitational acceleration
JRV  = 2.5;                  % [kg*m^2]  Moment of inertia of the front axle wheels
JRH  = 2.5;                  % [kg*m^2]  Moment of inertia of the rear axle wheels
hS = 0.5;                    % [m]       Height of the total center of gravity above the road surface
hA = 0.65;                   % [m]       Height of the vehicle body center of gravity above the road surface
l = 2.82;                    % [m]       Wheelbase
lV = 0.48*l;                 % [m]       Distance from the total center of gravity to the front axle
lH = l-lV;                   % [m]       Distance from the total center of gravity to the rear axle
lAV = 0.49*l;                % [m]       Distance from the vehicle body center of gravity to the front axle
lAH = l-lAV;                 % [m]       Distance from the vehicle body center of gravity to the rear axle
m = mA + mRV + mRH;          % [kg]      Vehicle mass
alpha_y = atan(qSt/100);     % [rad]     Inclination angle

PhiA = 0.635;                % [-]       Drive torque distribution
PhiB = 0.5;                  % [-]       Brake torque distribution

% Load tire parameters:
[tpV,tpH] = ReifenModell_1D;

% State vector:
% xvec(1);  % [m/s]     Vehicle velocity
% xvec(2);  % [rad/s]   Angular velocity of the front wheel
% xvec(3);  % [rad/s]   Angular velocity of the rear wheel
% xvec(4);  % [m]       Longitudinal tire deformation of the front wheel
% xvec(5);  % [m]       Longitudinal tire deformation of the rear wheel
% xvec(6);  % [m]       Vehicle position

xvec0 = [ 0; 0; 0; 0; 0; 0];    % Initial condition
tEnd = 10;                       % End time

% Integration of the differential equations
tic % Time measurement
[t,X] = ode45(@LDyn_Ex2,[0,tEnd],xvec0);
toc

% Extract some state variables:
vx = X(:,1);      % Velocity of the wheel center
omegaV = X(:,2);  % Angular velocity of the wheel
omegaH = X(:,3);  % Angular velocity of the wheel
xEV = X(:,4);     % Longitudinal tire deformation at the front axle
xEH = X(:,5);     % Longitudinal tire deformation at the rear axle
x = X(:,6);       % Vehicle position

% Determine additional output variables (somewhat cumbersome in MATLAB!)
MA = zeros(length(t),1);
MB = zeros(length(t),1);
lambda_xV = zeros(length(t),1);
lambda_xH = zeros(length(t),1);
FxV = zeros(length(t),1);
FxH = zeros(length(t),1);
FzV = zeros(length(t),1);
FzH = zeros(length(t),1);
ax = zeros(length(t),1);
nIter = zeros(length(t),1);
muM_xV = zeros(length(t),1);
muM_xH = zeros(length(t),1);
xEV_stat = zeros(length(t),1);
xEH_stat = zeros(length(t),1);
lambda0_xV = zeros(length(t),1);
lambda0_xH = zeros(length(t),1);
lambda_MxV = zeros(length(t),1);
lambda_MxH = zeros(length(t),1);

for k=1:length(t)
    [xp,zusatzwerte] = LDyn_Ex2(t(k),X(k,:)');
    MA(k) = zusatzwerte.MA;
    MBV(k) = zusatzwerte.MBV;
    MBH(k) = zusatzwerte.MBH;
    ax(k) = xp(1);
    lambda_xV(k) = zusatzwerte.lambda_xV;
    lambda_xH(k) = zusatzwerte.lambda_xH;
    FxV(k) = zusatzwerte.FxV;
    FxH(k) = zusatzwerte.FxH;
    FzV(k) = zusatzwerte.FzV;
    FzH(k) = zusatzwerte.FzH;
    nIter(k) = zusatzwerte.nIter;
    muM_xV(k) = zusatzwerte.muM_xV;
    muM_xH(k) = zusatzwerte.muM_xH;
    lambda_MxV(k) = zusatzwerte.lambda_MxV;
    lambda_MxH(k) = zusatzwerte.lambda_MxH;
    lambda0_xV(k) = zusatzwerte.lambda0_xV;
    lambda0_xH(k) = zusatzwerte.lambda0_xH;
    xEV_stat(k) = zusatzwerte.xEV_stat;
    xEH_stat(k) = zusatzwerte.xEH_stat;
end

% Equivalent road gradient ========================================================
qE = tan(alpha_y) + ax/(g*cos(alpha_y));
mu_xV = FxV./FzV;
mu_xH = FxH./FzH;

% Maximum equivalent road gradient
mu_max = (muM_xV+muM_xH)/2;
PhiA_grenz = (lV+hS*mu_max)/l;

for k=1:length(t)
    if PhiA<PhiA_grenz(k) 
        qEmax(k)  = lH/l/((1-PhiA)/mu_max(k) +hS/l);
    elseif abs(PhiA - PhiA_grenz(k)) < 1e-8
        qEmax(k)  = mu_max(k);
    else
        qEmax(k)  = lV/l/(PhiA./mu_max(k) -hS/l);
    end
end

% Graphical presentation of the results
figure
subplot(2,3,1); hold on, grid on
plot(t,vx,'Linewidth',1.5)
plot(t,tpV.rdyn*omegaV,'-.','Linewidth',1.5)
plot(t,tpV.rdyn*omegaH,'-.','Linewidth',1.5)
xlabel('Time t [s]')
ylabel('Velocities v_x and v_u at the front / rear axle [m/s]')
title('Velocities (v_u=\omega r_{dyn})')
legend('v_x','v_u (front axle)','v_u (rear axle)')
xlabel('Time t [s]')

subplot(2,3,2); hold on, grid on
plot(t,x,'Linewidth',1.5)
title('Vehicle position')
xlabel('Time t [s]')
ylabel('Distance x(t) [m]')

subplot(2,3,3); hold on, grid on
plot(t,MA,'Linewidth',1.5)
plot(t,MBV,'-.','Linewidth',1.5)
plot(t,MBH,'-.','Linewidth',1.5)
legend('M_A','M_{BV}','M_{BH}')
title('Drive and brake torque')
xlabel('Time t [s]')
ylabel('Torques M_A and M_B [Nm]')

subplot(2,3,4); hold on, grid on
if ~verLessThan('matlab', '9.0')
    yyaxis left
end
plot(t,FxV,'Linewidth',1.5)
ylabel('F_{xV} [N]')
if ~verLessThan('matlab', '9.0')
    yyaxis right
end
plot(t,lambda0_xV,'Linewidth',1.5)
xlabel('Time t [s]')
ylabel('\lambda_{xV}^0 [-]')
title('Longitudinal force and longitudinal slip (front axle)')

subplot(2,3,5); hold on, grid on
if ~verLessThan('matlab', '9.0')
    yyaxis left
end
plot(t,FxH,'Linewidth',1.5)
ylabel('F_{xH} [N]')
if ~verLessThan('matlab', '9.0')
    yyaxis right
end
plot(t,lambda0_xH,'Linewidth',1.5)
xlabel('Time t [s]')
ylabel('\lambda_{xH}^0 [-]')
title('Longitudinal force and longitudinal slip (rear axle)')

subplot(2,3,6); hold on, grid on
plot(t,FzV,'Linewidth',1.5)
plot(t,FzH,'Linewidth',1.5)
xlabel('Time t [s]')
ylabel('F_{zV} [N] and F_{zH} [N]')
title('Wheel loads at the front and rear axles')
legend('F_{zV} [N]','F_{zH} [N]')

figure 
subplot(2,3,1); hold on, grid on
plot(t,qE,'Linewidth',1.5)
plot(t,qEmax,'r--','Linewidth',1.5)
xlabel('Time t [s]')
title('Equivalent road gradient and approximation of maximum equivalent gradient')
legend('q_E [-]','q_{Emax} [-]')

subplot(2,3,2); hold on, grid on
plot(t,mu_xV,'k','Linewidth',1.5)
plot(t,muM_xV,'k--','Linewidth',1.5)
plot(t,-muM_xV,'k--','Linewidth',1.5)
xlabel('Time t [s]')
title('Tire-road friction utilization at the front axle')
legend('\mu_{xV}','\mu_{MxV}')
xlabel('Time t [s]')

subplot(2,3,3); hold on, grid on
plot(t,mu_xH,'k','Linewidth',1.5)
plot(t,muM_xH,'k--','Linewidth',1.5)
plot(t,-muM_xH,'k--','Linewidth',1.5)
xlabel('Time t [s]')
title('Tire-road friction utilization at the rear axle')
legend('\mu_{xH}','\mu_{MxH}')
xlabel('Time t [s]')

subplot(2,3,4); hold on, grid on
plot(t,ax,'k','Linewidth',1.5)
title('Acceleration')
xlabel('Time t [s]')
ylabel('a_x [m/s^2]')

subplot(2,3,5); hold on, grid on
plot(t,xEV,'b','Linewidth',1.5)
plot(t,xEH,'r','Linewidth',1.5)
plot(t,xEV_stat,'b--','Linewidth',1.5)
plot(t,xEH_stat,'r--','Linewidth',1.5)
title('Tire deformation at the front and rear axles')
legend('x_{EV}','x_{EH}','x_{EV}^{stat}','x_{EH}^{stat}')
xlabel('Time t [s]')

subplot(2,3,6); hold on, grid on
plot(t,nIter,'Linewidth',1.5)
xlabel('Time t [s]')
title(['Number of iterations (Maximum:' , num2str(maxIter),')'])
legend('nIter [-]')
xlabel('Time t [s]')
